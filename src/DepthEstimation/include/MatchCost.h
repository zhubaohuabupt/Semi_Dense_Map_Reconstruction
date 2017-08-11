#include"opencv2/opencv.hpp"
#include"cv.h"
#include<vector>
using namespace std;

namespace  MatchCost
{
class Cost{
public:
    virtual  float calCost(uint8_t*cur_patch,uint8_t* ref_patch,int center)=0;
};
class CT:public Cost//CT代价
{
public:
    vector<bool> calCT(uint8_t* patch,int center)
	{
		vector<bool> CTresult;
		for(int i=0;i<=2*center;i++)
		{
		if(i==center) continue;
		 bool binary= patch[i]-patch[center]>0?1:0;
		   CTresult.push_back(binary);
		}
		return CTresult;

	}

    int calDif(const vector <bool>&cur_patch_binary,const vector <bool>&ref_patch_binary)
	{
		int  dif=0;
		for(int i=0;i<cur_patch_binary.size();i++)
		{
		 if(cur_patch_binary[i]!=ref_patch_binary[i])
			dif++;	
		}
	return dif;
	}
    float calCost(uint8_t*cur_patch,uint8_t* ref_patch,int center)
	{
    vector <bool> cur_patch_binary =calCT(cur_patch,center);
    vector <bool> ref_patch_binary =calCT(ref_patch,center);
	return calDif(cur_patch_binary,ref_patch_binary);
	}

};

class ZMSAD:public Cost{
public:
      float calMean(uint8_t* patch,int center)
      {
          double mean=0;
          for(int i=0;i<=2*center;i++)
            mean+=patch[i];
         return mean/(center*2);
      }
float calCost(uint8_t*cur_patch,uint8_t* ref_patch,int center)
    {
        float cur_mean=calMean(cur_patch,center);
        float ref_mean=calMean(ref_patch,center);
        double zmsad=0;
    for(int i=0;i<=2*center;i++)
            zmsad=abs(cur_patch[i]-cur_mean-ref_patch[i]+ref_mean);

        return zmsad;
    }
};

 class ZNCC:public Cost
   {
 public:
     float calMean(uint8_t* patch,int center)
         {
             double mean=0;
             for(int i=0;i<=2*center;i++)
               mean+=patch[i];
            return mean/(center*2);
         }
     double calVal(uint8_t* patch,float mean,int center)
     {
         double Val=0;
         for(int i=0;i<=2*center;i++)
           Val+=(patch[i]-mean)*(patch[i]-mean);
        return sqrtf(Val);
     }
    float calCost(uint8_t*cur_patch,uint8_t* ref_patch,int center)
        {
        float cur_mean=calMean(cur_patch,center);
        float ref_mean=calMean(ref_patch,center);

        double  cur_Val=calVal(cur_patch,cur_mean,center);
         double  ref_Val=calVal(ref_patch,ref_mean,center);

         float zncc=0;
            for(int i=0;i<=2*center;i++)
                zncc+=(cur_patch[i]-cur_mean)*(ref_patch[i]-ref_mean);

         return  zncc/=(cur_Val*ref_Val);
        }

 };
}
