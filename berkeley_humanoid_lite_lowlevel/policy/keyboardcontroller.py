import threading

import pynput


class KeyboardController:
    def __init__(self):
        self._listener = pynput.keyboard.Listener(on_press=self.on_key_press, on_release=self.on_key_release)
        self._is_stopped = threading.Event()
        self._is_stopped.clear()
        self.key_states = {}

    def get(self, key):
        return self.key_states.get(key.upper())

    def on_key_press(self, key):
        if not key:
            return
        if type(key) == pynput.keyboard.KeyCode:
            kc = key.char.upper()
            self.key_states[kc] = True
            if kc == "1":
                self.on1Pressed()
            elif kc == "2":
                self.on2Pressed()
            elif kc == "3":
                self.on3Pressed()
            elif kc == "4":
                self.on4Pressed()
            elif kc == "5":
                self.on5Pressed()
            elif kc == "6":
                self.on6Pressed()
            elif kc == "7":
                self.on7Pressed()
            elif kc == "8":
                self.on8Pressed()
            elif kc == "9":
                self.on9Pressed()
            elif kc == "0":
                self.on0Pressed()
            elif kc.upper() == "A":
                self.onAPressed()
            elif kc.upper() == "B":
                self.onBPressed()
            elif kc.upper() == "X":
                self.onXPressed()
            elif kc.upper() == "Y":
                self.onYPressed()
        else:
            if key == pynput.keyboard.Key.esc:
                self.key_states["ESC"] = True
                self.onESCPressed()
            elif key == pynput.keyboard.Key.left:
                self.key_states["LEFT"] = True
                self.onLeftArrowPressed()
            elif key == pynput.keyboard.Key.right:
                self.key_states["RIGHT"] = True
                self.onLeftArrowPressed()
            elif key == pynput.keyboard.Key.up:
                self.key_states["UP"] = True
                self.onUpArrowPressed()
            elif key == pynput.keyboard.Key.down:
                self.key_states["DOWN"] = True
                self.onDownArrowPressed()
        
        return not self._is_stopped.is_set()

    def on_key_release(self, key):
        if type(key) == pynput.keyboard._xorg.KeyCode:
            kc = key.char.upper()
            self.key_states[kc] = False
            # if kc == "1":
            #     self.on1Pressed()
            # elif kc == "2":
            #     self.on2Pressed()
            # elif kc == "3":
            #     self.on3Pressed()
            # elif kc == "4":
            #     self.on4Pressed()
            # elif kc == "5":
            #     self.on5Pressed()
            # elif kc == "6":
            #     self.on6Pressed()
            # elif kc == "7":
            #     self.on7Pressed()
            # elif kc == "8":
            #     self.on8Pressed()
            # elif kc == "9":
            #     self.on9Pressed()
            # elif kc == "0":
            #     self.on0Pressed()
            # elif kc.upper() == "A":
            #     self.onAPressed()
            # elif kc.upper() == "B":
            #     self.onBPressed()
            # elif kc.upper() == "X":
            #     self.onXPressed()
            # elif kc.upper() == "Y":
            #     self.onYPressed()
        else:
            if key == pynput.keyboard.Key.esc:
                self.key_states["ESC"] = False
                # self.onESCPressed()
            elif key == pynput.keyboard.Key.left:
                self.key_states["LEFT"] = False
                # self.onLeftArrowPressed()
            elif key == pynput.keyboard.Key.right:
                self.key_states["RIGHT"] = False
                # self.onLeftArrowPressed()
            elif key == pynput.keyboard.Key.up:
                self.key_states["UP"] = False
                # self.onUpArrowPressed()
            elif key == pynput.keyboard.Key.down:
                self.key_states["DOWN"] = False
                # self.onDownArrowPressed()
        
        return not self._is_stopped.is_set()

    def start(self):
        self._listener.start()

    def stop(self):
        self._is_stopped.set()
        self._listener.join()
    
    def onESCPressed(self):
        pass
    def on1Pressed(self):
        pass
    def on2Pressed(self):
        pass
    def on3Pressed(self):
        pass
    def on4Pressed(self):
        pass
    def on5Pressed(self):
        pass
    def on6Pressed(self):
        pass
    def on7Pressed(self):
        pass
    def on8Pressed(self):
        pass
    def on9Pressed(self):
        pass
    def on0Pressed(self):
        pass
    def onAPressed(self):
        pass
    def onBPressed(self):
        pass
    def onXPressed(self):
        pass
    def onYPressed(self):
        pass
    def onLeftArrowPressed(self):
        pass
    def onRightArrowPressed(self):
        pass
    def onUpArrowPressed(self):
        pass
    def onDownArrowPressed(self):
        pass

if __name__ == "__main__":
    keycontroller = KeyboardController()
    keycontroller.start()

    keycontroller.onAPressed = lambda: print("A pressed")
    keycontroller.onBPressed = lambda: print("B pressed")
    keycontroller.onXPressed = lambda: print("X pressed")
    keycontroller.onYPressed = lambda: print("Y pressed")
    keycontroller.onLeftArrowPressed = lambda: print("<- pressed")
    keycontroller.onRightArrowPressed = lambda: print("-> pressed")

    try:
        while True:
            pass
    except KeyboardInterrupt:
        pass

    keycontroller.stop()
