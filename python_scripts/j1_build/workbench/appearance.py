"""Display-only studio lighting and contact shading; scene units are meters."""
import pyvista as pv
from .config import DisplayOptions

DISPLAY = DisplayOptions()
PHANTOM_COLOR = DISPLAY.phantom_color
PHANTOM_OPACITY = DISPLAY.phantom_opacity
SURFACE_MATERIAL = dict(ambient=DISPLAY.surface_ambient, diffuse=DISPLAY.surface_diffuse,
    specular=DISPLAY.surface_specular, specular_power=DISPLAY.surface_specular_power)


def configure_studio_lighting(plotter, intensity=None):
    multiplier = DISPLAY.light_intensity if intensity is None else intensity
    plotter.remove_all_lights()
    for position, strength in [((-0.6, 0.8, 1.0), DISPLAY.key_light_intensity),
            ((0.8, 0.1, 0.5), DISPLAY.fill_light_intensity), ((0.0, -0.7, -0.6), DISPLAY.rim_light_intensity)]:
        plotter.add_light(pv.Light(position=position, focal_point=(0, 0, 0),
            color="white", intensity=strength * multiplier, light_type="camera light", positional=False))


def set_contact_shading(plotter, enabled):
    if enabled:
        # SSAO darkens crevices and contacts without changing mesh geometry.
        plotter.enable_ssao(radius=DISPLAY.ssao_radius_mm * 0.001,
            bias=DISPLAY.ssao_bias_mm * 0.001, kernel_size=DISPLAY.ssao_kernel_size, blur=True)
    else:
        plotter.disable_ssao()
