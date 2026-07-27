"""Tests for chess_fritz.publisher.ChessMovePublisher.

pika is mocked via an injectable connection-factory parameter so these
tests never touch a real broker.
"""
import logging

import pika
import pika.exceptions
import pytest

from chess_fritz.publisher import ChessMovePublisher


class FakeChannel:
    def __init__(self):
        self.confirm_delivery_called = False
        self.published = []
        self.publish_side_effects = []
        self.declared_exchange = None
        self.declared_queue = None
        self.bound = None

    def confirm_delivery(self):
        self.confirm_delivery_called = True

    def exchange_declare(self, exchange, exchange_type):
        self.declared_exchange = (exchange, exchange_type)

    def queue_declare(self, queue):
        self.declared_queue = queue

    def queue_bind(self, exchange, queue, routing_key):
        self.bound = (exchange, queue, routing_key)

    def basic_publish(self, **kwargs):
        if self.publish_side_effects:
            effect = self.publish_side_effects.pop(0)
            if effect is not None:
                raise effect
        self.published.append(kwargs)


class FakeConnection:
    def __init__(self, params=None):
        self.params = params
        self.channel_obj = FakeChannel()
        self._closed = False

    def channel(self):
        return self.channel_obj

    @property
    def is_closed(self):
        return self._closed

    def close(self):
        self._closed = True


class FakeConnectionFactory:
    """Records every connection created; each gets a fresh FakeChannel."""

    def __init__(self, publish_side_effects=None):
        self.connections = []
        # side effects queued onto every newly created channel, in order
        self._pending_publish_side_effects = list(publish_side_effects or [])

    def __call__(self, params):
        conn = FakeConnection(params)
        conn.channel_obj.publish_side_effects = self._pending_publish_side_effects
        self.connections.append(conn)
        return conn


def make_publisher(factory=None, monkeypatch=None):
    if factory is None:
        factory = FakeConnectionFactory()
    if monkeypatch is not None:
        monkeypatch.setattr("chess_fritz.publisher.time.sleep", lambda *_: None)
    logger = logging.getLogger("test")
    publisher = ChessMovePublisher(logger, connection_factory=factory)
    return publisher, factory


def test_setup_connection_uses_config_host_port_heartbeat(monkeypatch):
    publisher, factory = make_publisher(monkeypatch=monkeypatch)
    params = factory.connections[0].params
    assert params.host == "localhost"  # host_from_windows in messaging_config.yaml
    assert params.port == 5672
    assert params.heartbeat == 60


def test_confirm_delivery_enabled_on_channel(monkeypatch):
    publisher, factory = make_publisher(monkeypatch=monkeypatch)
    assert factory.connections[0].channel_obj.confirm_delivery_called is True


def test_publish_move_returns_true_on_success(monkeypatch):
    publisher, factory = make_publisher(monkeypatch=monkeypatch)
    ok = publisher.publish_move("e2e4", "white")
    assert ok is True
    published = factory.connections[0].channel_obj.published
    assert len(published) == 1
    import json
    body = json.loads(published[0]["body"])
    assert body == {"from_square": "e2", "to_square": "e4"}


def test_publish_move_retries_and_recovers(monkeypatch):
    # First publish attempt fails transiently; reconnect creates a fresh
    # connection/channel, second attempt on the new channel succeeds.
    factory = FakeConnectionFactory(publish_side_effects=[ConnectionError("boom")])
    publisher, factory = make_publisher(factory=factory, monkeypatch=monkeypatch)

    ok = publisher.publish_move("e2e4", "white")

    assert ok is True
    # A reconnection happened: a second connection was created.
    assert len(factory.connections) == 2
    assert len(factory.connections[1].channel_obj.published) == 1


def test_publish_move_never_raises_and_returns_false_after_max_retries(monkeypatch):
    # Every attempt fails -- publish_move must exhaust retries and return
    # False rather than propagating the exception (B6: survive double failure).
    always_fail = [ConnectionError("boom")] * 10
    factory = FakeConnectionFactory(publish_side_effects=always_fail)
    publisher, factory = make_publisher(factory=factory, monkeypatch=monkeypatch)

    ok = publisher.publish_move("e2e4", "white")

    assert ok is False


def test_publish_move_returns_false_on_unroutable(monkeypatch):
    unroutable = pika.exceptions.UnroutableError([])
    factory = FakeConnectionFactory(publish_side_effects=[unroutable])
    publisher, factory = make_publisher(factory=factory, monkeypatch=monkeypatch)

    ok = publisher.publish_move("e2e4", "white")

    assert ok is False


def test_cleanup_closes_open_connection(monkeypatch):
    publisher, factory = make_publisher(monkeypatch=monkeypatch)
    publisher.cleanup()
    assert factory.connections[0].is_closed is True


def test_publish_reset_sends_reset_body(monkeypatch):
    publisher, factory = make_publisher(monkeypatch=monkeypatch)
    ok = publisher.publish_reset()
    assert ok is True
    published = factory.connections[0].channel_obj.published
    assert len(published) == 1
    import json
    body = json.loads(published[0]["body"])
    assert body == {"type": "reset"}
