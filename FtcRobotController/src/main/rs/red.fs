#pragma version(1)
#pragma rs java_package_name(com.qualcomm.ftcrobotcontroller)
#pragma RS_FP_IMPRECISE

uchar4 __attribute__ ((kernel)) split(uchar4 in, uint32_t x, uint32_t y){

    //rgb filter=========================================================
    //IN.B IS RED AND IN.R IS BLUE
    uchar4 pixelOut;
    if(in.b>in.g+20&&in.b>in.r+20){
        pixelOut=(uchar4){255,0,0,255};
    }else{
        pixelOut=(uchar4){0,0,0,255};
   }
   return pixelOut;

}




