import logging
import pywinauto
from pywinauto import Application
from typing import Optional, List
import pyperclip
import time
import warnings

# Suppress pywinauto UserWarning about key combinations
warnings.filterwarnings("ignore", category=UserWarning, 
                       message="Key combinations may or may not work depending on the target app")

class FritzWindowHandler:
    def __init__(self, logger: logging.Logger):
        self.logger = logger
        self.fritz_window = None
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

        except Exception as e:
            self.logger.error(f"Failed to connect to Fritz: {e}")
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
        """Copy game from Fritz to clipboard"""
        try:
            self.fritz_window.set_focus()
            self.fritz_window.send_keystrokes('^c')
            time.sleep(0.1)  # Wait for clipboard to update
            return pyperclip.paste()
        except Exception as e:
            self.logger.error(f"Failed to copy game: {e}")
            return None
