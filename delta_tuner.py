from kivy.app import App
from kivy.uix.widget import Widget
from kivy.core.window import Window
from kivy.modules import inspector
from kivy.uix.relativelayout import RelativeLayout
from kivy.properties import NumericProperty

Window.size = (700, 420)

class DeltaTunerMain(Widget):
    pass


class DeltaTunerApp(App):
    # icon = 'custom-kivy-icon.png'
    title = "Delta Tuner"
    def build(self):
        dt = DeltaTunerMain()
        inspector.create_inspector(Window, dt)
        return dt

class DTErrorBar(RelativeLayout):
    pass

class DTErrorMarker(RelativeLayout):
    value = NumericProperty(0)

if __name__ == "__main__":
    DeltaTunerApp().run()