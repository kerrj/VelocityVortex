#pragma version(1)
#pragma rs java_package_name(com.qualcomm.ftcrobotcontroller)
#pragma RS_FP_IMPRECISE

uchar __attribute__ ((kernel)) split(uchar4 in, uint32_t x, uint32_t y){

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
            H=(float)60.0f*( fmod((G-B)/delta,6));
        }else if(Cmax==G){
            H=(float)60.0f*( ((B-R)/delta)+2);
        }else if(Cmax==B){
            H=(float)60.0f*( ((R-G)/delta)+4);
        }
        float V=Cmax;
        float S=delta/V;
        //if(x==400&&y==240){
          //  rsDebug("Cmax",Cmax);
         //   rsDebug("Cmin",Cmin);
          //  rsDebug("delta",delta);
          //  rsDebug("R",R);
          //  rsDebug("G",G);
          //  rsDebug("B",B);
            //rsDebug("H",H);
        //}

        uchar pixelOut;
        //if(H>170&&H<270&&V>.2f&&S>.5f){//blue
         //   pixelOut=(uchar){(in.b+in.r+in.g)/3};
        //}else{
        //    pixelOut=(uchar){255};
        //}
        return pixelOut;

}




