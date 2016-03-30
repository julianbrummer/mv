#ifdef GL_ES
// Set default precision to medium
precision mediump int;
precision mediump float;
#endif

uniform vec4 uColor;
varying float vIntensity;

void main()
{
    /*
    if (gl_FrontFacing)
        gl_FragColor = vIntensity*vec4(0.0,0.0,1.0,0.0);
    else
        gl_FragColor = vIntensity*vec4(1.0,0.0,0.0,0.0);
    */
    gl_FragColor = vIntensity*uColor;
}

