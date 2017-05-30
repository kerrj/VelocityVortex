#pragma version(1)
#pragma rs java_package_name(com.qualcomm.ftcrobotcontroller)
#pragma RS_FP_IMPRECISE

uchar3 rgb;

uchar4 __attribute__ ((kernel)) split(uchar4 in, uint32_t x, uint32_t y){

    //rgb filter=========================================================
    uchar4 pixelOut;
    if(in.b>in.g&&in.b>in.r+20){
        pixelOut=(uchar4){0,0,255,255};
    }else if(in.r>in.g&&in.r>in.b+20){
        pixelOut=(uchar4){255,0,0,255};
    }else{
        pixelOut=(uchar4){0,0,0,255};
   }
   return pixelOut;
    //hsv filter========================================

    float R=(float)in.r/255.000000f;
    float G=(float)in.g/255.000000f;
    float B=(float)in.b/255.000000f;

    float Cmax=fmax(R,G);
    Cmax=fmax(Cmax,B);

    float Cmin=fmin(R,G);
    Cmin=fmin(Cmin,B);

    float delta=(float)Cmax-Cmin;

    float H;
    if(delta==0.0f){
        H=0;
    }else if(Cmax==R){
        H=(float)60.0f*(fmod((G-B)/delta,6));
    }else if(Cmax==G){
        H=(float)60.00f*( ((B-R)/delta)+2);
    }else if(Cmax==B){
        H=(float)60.000000f*( ((R-G)/delta)+4);
    }
     float V=Cmax;
     float S=delta/V;

    uchar4 pixelOut;
    if(H>345||H<15){
        pixelOut=(uchar4){255,255,255,255};
    }else{
        pixelOut=(uchar4){0,0,0,255};
    }
    return pixelOut;
}




