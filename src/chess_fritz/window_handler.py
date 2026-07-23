import logging
import pywinauto
from pywinauto import Application
from typing import Optional, List
import pyperclip
import time
import warnings

from chess_common.config import load_config

# Suppress pywinauto UserWarning about key combinations
warnings.filterwarnings("ignore", category=UserWarning,
                       message="Key combinations may or may not work depending on the target app")

class FritzWindowHandler:
    def __init__(self, logger: logging.Logger):
        self.logger = logger
        self.fritz_window = None
        fritz_cfg = load_config('fritz_config')['fritz']
        self.clipboard_delay_sec = fritz_cfg['clipboard_delay_sec']
        self.reconnect_after_failures = fritz_cfg['reconnect_after_failures']
        self._consecutive_failures = 0
        self._connect_to_fritz()

    def _connect_to_fritz(self) -> None:
        """Connect to Fritz chess interface"""
        try:
            windows = pywinauto.findwindows.find_elements(title_re=".*Fritz.*")
            if not windows:
                raise RuntimeError("No Fritz windows found")

            self.logger.info("Found Fritz windows:")
            for i, win in enumerate(windows):
                self.logger.info(f"{i + 1}: {win.name}")

            selected_window = self._select_window(windows)
            self.app = Application(backend="win32").connect(handle=selected_window.handle)
            self.fritz_window = self.app.window(handle=selected_window.handle)
            self.logger.info(f"Connected to Fritz window: {selected_window.name}")

        except Exception:
            self.logger.error("Failed to connect to Fritz", exc_info=True)
            raise

    def _select_window(self, windows: List) -> pywinauto.WindowSpecification:
        """Handle Fritz window selection"""
        if len(windows) == 1:
            return windows[0]

        while True:
            try:
                choice = int(input(f"\nMultiple Fritz windows found. Select (1-{len(windows)}): "))
                if 1 <= choice <= len(windows):
                    return windows[choice - 1]
                print(f"Please enter a number between 1 and {len(windows)}")
            except ValueError:
                print("Please enter a valid number")

    def copy_game_to_clipboard(self) -> Optional[str]:
        """Copy game from Fritz to clipboard.

        After ``reconnect_after_failures`` consecutive failures, attempts a
        fresh window search/connect (the Fritz window handle may have gone
        stale -- e.g. window closed/reopened) before giving up on this poll.
        """
        try:
            self.fritz_window.set_focus()
            self.fritz_window.send_keystrokes('^c')
            time.sleep(self.clipboard_delay_sec)  # Wait for clipboard to update
            content = pyperclip.paste()
            self._consecutive_failures = 0
            return content
        except Exception:
            self._consecutive_failures += 1
            self.logger.error(
                "Failed to copy game (%d consecutive failure(s))",
                self._consecutive_failures, exc_info=True,
            )
            if self._consecutive_failures >= self.reconnect_after_failures:
                self.logger.info(
                    "Reconnecting to Fritz after %d consecutive failures",
                    self._consecutive_failures,
                )
                try:
                    self._connect_to_fritz()
                    self._consecutive_failures = 0
                except Exception:
                    self.logger.error("Reconnect attempt failed", exc_info=True)
            return None
