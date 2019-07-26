attribute vec4 position;
attribute vec3 color;

varying vec3 vposition;
varying vec3 vcolor;

void main () {
    gl_Position = position;
    vcolor = color;
    vposition = position.xyz;
}
