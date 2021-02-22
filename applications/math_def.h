#ifndef __MATH_DEF_H__
#define __MATH_DEF_H__

#define RAD_TO_DEG 57.2957795f
#define DEG_TO_RAD 0.01745329f


#define min(a,b) ((a<b)?a:b)
#define max(a,b) ((a>b)?a:b)
#define confine(amt,low,high)     (max(min(amt, high),low))

#define abs(a) a>0?a:-a

#endif