#ifndef __RAMP_CONTROLLER__
#define __RAMP_CONTROLLER__
#include "types.hpp"

namespace UserLib
{
    class Ramp
    {
       public:
        explicit Ramp(fp32 acc, fp32 dt);
        ~Ramp() = default;
        void update(fp32 input);
        void setAcc(fp32 acc_t);
        void clear();
        void clear(fp32 set_v);

       public:
        fp32 out = 0.f;

       private:
        fp32 acc = 0.f; //加速度
        fp32 dt = 0.f;
    };

}  // namespace Ramp

#endif
