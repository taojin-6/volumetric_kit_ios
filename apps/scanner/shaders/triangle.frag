#version 450

// Pass the interpolated vertex colour through. Interpolation across the
// triangle is itself part of what this proves: a flat fill could come from a
// clear, whereas a gradient can only come from the rasteriser.

layout(location = 0) in vec3 v_color;
layout(location = 0) out vec4 out_color;

void main() { out_color = vec4(v_color, 1.0); }
