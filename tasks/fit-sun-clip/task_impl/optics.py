"""Transparent and tinted optical surfaces on the actual collision geometry."""

def bind_visible_surface(binding, body, index):
    original = f"rel material:binding = </Fixture/Materials/{body}>"
    if binding.count(original) != 1:
        raise ValueError("Expected one visible surface binding")
    return binding.replace(original, f"rel material:binding = </Fixture/Materials/optical_{body}_{index}>")


def emit_materials(actors):
    from .scene import identifier, number, triple
    materials = []
    for actor in actors:
        for index, part in enumerate(actor["parts"]):
            surface = part.get("optical_surface")
            if surface is None:
                continue
            name = f"optical_{identifier(actor['name'])}_{index}"
            materials.append(f'''
    def Material "{name}"
    {{
        token outputs:surface.connect = </Fixture/Materials/{name}/Surface.outputs:surface>
        def Shader "Surface"
        {{
            uniform token info:id = "UsdPreviewSurface"
            color3f inputs:diffuseColor = {triple(surface["rgb"])}
            float inputs:opacity = {number(surface["opacity"])}
            float inputs:roughness = 0.12
            float inputs:ior = 1.52
            token outputs:surface
        }}
    }}''')
    return "".join(materials)
