# 枪口射频前馈控制
对于枪口射速的控制，可以引入前馈控制。引入前馈之后可以根据射频计算当前等级下的可以持续射击的时间，或者根据想要持续射击的时间选择射频。
# 依据：
由于裁判系统每 100ms（0.1s）结算一次，且为先结算然后再冷却：
![image](https://cdn-mineru.openxlab.org.cn/result/2026-03-10/f368bffd-a4d7-4c28-bc16-9a7605d11775/bb6a7695a00420f24f3db7aa3c8623e7bbb2828dcec6b084b42a7208da3b64e8.jpg)


图5-3枪口热量冷却逻辑图

假设当前机器人冷却??（/s），枪口热量上限 $m$ ，射速??（发/秒），每射击一发消耗热量 d，射击持续时间 $t = \frac { n } { 1 0 }$ （s， $n \in N )$ ）。
所以为了保证在每个 100ms 时间都不会超热量所以有：
$$
d c t \leq m + a (t - \frac {1}{1 0})
$$
即：
$$
\frac {d}{1 0} c n \leq m + a (\frac {t - 1}{1 0})
$$
可以计算出：
$$
\left(c - \frac {a}{d}\right) n \leq \frac {1 0 m}{d} - \frac {a}{d}
$$
当 $c \leq \frac { a } { 1 0 }$ 时，即当前射频造成的热量消耗小于等于冷却，此情况不可能超热量，故只考虑 $c \geq$ ?? 10时的情况： a
$$
n \leq \frac {1 0 m - a}{c d - a} \dots \dots (1)
$$
或：
$$
c \leq \frac {1 0 m - a}{d n} + \frac {a}{d} \dots \dots (2)
$$
然后我们就可以根据（1）式和（2）式来规划我们的射击方案。
例如前 t 秒高射频，之后将为热量消耗和冷却速度一致。
这样一个方案会在这 t 秒内造成一定的热量浪费，在 2024 赛季规则下（ $\mathrm { d } = 1 0$ ），具体如下：
（使用 python 计算）

输入尿干时间(s):2

<table><tr><td>爆发优先(level 1):</td><td>射速（发/秒）:10.95;</td><td>浪费热量: 1.00;</td><td>热量浪费比率: 0.45%</td></tr><tr><td>爆发优先(level 2):</td><td>射速（发/秒）:13.93;</td><td>浪费热量: 1.50;</td><td>热量浪费比率: 0.54%</td></tr><tr><td>爆发优先(level 3):</td><td>射速（发/秒）:16.90;</td><td>浪费热量: 2.00;</td><td>热量浪费比率: 0.59%</td></tr><tr><td>爆发优先(level 4):</td><td>射速（发/秒）:19.88;</td><td>浪费热量: 2.50;</td><td>热量浪费比率: 0.62%</td></tr><tr><td>爆发优先(level 5):</td><td>射速（发/秒）:22.85;</td><td>浪费热量: 3.00;</td><td>热量浪费比率: 0.65%</td></tr><tr><td>爆发优先(level 6):</td><td>射速（发/秒）:25.82;</td><td>浪费热量: 3.50;</td><td>热量浪费比率: 0.67%</td></tr><tr><td>爆发优先(level 7):</td><td>射速（发/秒）:28.80;</td><td>浪费热量: 4.00;</td><td>热量浪费比率: 0.69%</td></tr><tr><td>爆发优先(level 8):</td><td>射速（发/秒）:31.77;</td><td>浪费热量: 4.50;</td><td>热量浪费比率: 0.70%</td></tr><tr><td>爆发优先(level 9):</td><td>射速（发/秒）:34.75;</td><td>浪费热量: 5.00;</td><td>热量浪费比率: 0.71%</td></tr><tr><td>爆发优先(level 10):</td><td>射速（发/秒）:38.20;</td><td>浪费热量: 6.00;</td><td>热量浪费比率: 0.78%</td></tr></table>
<table><tr><td>射速优先(level 1):</td><td>射速（发/秒）:6.30;</td><td>浪费热量:4.00;</td><td>热量浪费比率:3.08%</td></tr><tr><td>射速优先(level 2):</td><td>射速（发/秒）:8.53;</td><td>浪费热量:4.50;</td><td>热量浪费比率:2.57%</td></tr><tr><td>射速优先(level 3):</td><td>射速（发/秒）:10.75;</td><td>浪费热量:5.00;</td><td>热量浪费比率:2.27%</td></tr><tr><td>射速优先(level 4):</td><td>射速（发/秒）:12.97;</td><td>浪费热量:5.50;</td><td>热量浪费比率:2.08%</td></tr><tr><td>射速优先(level 5):</td><td>射速（发/秒）:15.20;</td><td>浪费热量:6.00;</td><td>热量浪费比率:1.94%</td></tr><tr><td>射速优先(level 6):</td><td>射速（发/秒）:17.43;</td><td>浪费热量:6.50;</td><td>热量浪费比率:1.83%</td></tr><tr><td>射速优先(level 7):</td><td>射速（发/秒）:19.65;</td><td>浪费热量:7.00;</td><td>热量浪费比率:1.75%</td></tr><tr><td>射速优先(level 8):</td><td>射速（发/秒）:21.88;</td><td>浪费热量:7.50;</td><td>热量浪费比率:1.69%</td></tr><tr><td>射速优先(level 9):</td><td>射速（发/秒）:24.10;</td><td>浪费热量:8.00;</td><td>热量浪费比率:1.63%</td></tr><tr><td>射速优先(level 10):</td><td>射速（发/秒）:27.60;</td><td>浪费热量:8.00;</td><td>热量浪费比率:1.43%</td></tr></table>

输入尿干时间(s):4

<table><tr><td>爆发优先(level 1):</td><td>射速（发/秒）:5.97;</td><td>浪费热量: 1.00;</td><td>热量浪费比率: 0.42%</td></tr><tr><td>爆发优先(level 2):</td><td>射速（发/秒）:7.71;</td><td>浪费热量: 1.50;</td><td>热量浪费比率: 0.48%</td></tr><tr><td>爆发优先(level 3):</td><td>射速（发/秒）:9.45;</td><td>浪费热量: 2.00;</td><td>热量浪费比率: 0.53%</td></tr><tr><td>爆发优先(level 4):</td><td>射速（发/秒）:11.19;</td><td>浪费热量: 2.50;</td><td>热量浪费比率: 0.56%</td></tr><tr><td>爆发优先(level 5):</td><td>射速（发/秒）:12.93;</td><td>浪费热量: 3.00;</td><td>热量浪费比率: 0.58%</td></tr><tr><td>爆发优先(level 6):</td><td>射速（发/秒）:14.66;</td><td>浪费热量: 3.50;</td><td>热量浪费比率: 0.59%</td></tr><tr><td>爆发优先(level 7):</td><td>射速（发/秒）:16.40;</td><td>浪费热量: 4.00;</td><td>热量浪费比率: 0.61%</td></tr><tr><td>爆发优先(level 8):</td><td>射速（发/秒）:18.14;</td><td>浪费热量: 4.50;</td><td>热量浪费比率: 0.62%</td></tr><tr><td>爆发优先(level 9):</td><td>射速（发/秒）:19.88;</td><td>浪费热量: 5.00;</td><td>热量浪费比率: 0.62%</td></tr><tr><td>爆发优先(level 10):</td><td>射速（发/秒）:22.10;</td><td>浪费热量: 6.00;</td><td>热量浪费比率: 0.67%</td></tr><tr><td>射速优先(level 1):</td><td>射速（发/秒）:5.15;</td><td>浪费热量:4.00;</td><td>热量浪费比率:1.90%</td></tr><tr><td>射速优先(level 2):</td><td>射速（发/秒）:6.51;</td><td>浪费热量:4.50;</td><td>热量浪费比率:1.70%</td></tr><tr><td>射速优先(level 3):</td><td>射速（发/秒）:7.88;</td><td>浪费热量:5.00;</td><td>热量浪费比率:1.56%</td></tr><tr><td>射速优先(level 4):</td><td>射速（发/秒）:9.24;</td><td>浪费热量:5.50;</td><td>热量浪费比率:1.47%</td></tr><tr><td>射速优先(level 5):</td><td>射速（发/秒）:10.60;</td><td>浪费热量:6.00;</td><td>热量浪费比率:1.40%</td></tr><tr><td>射速优先(level 6):</td><td>射速（发/秒）:11.96;</td><td>浪费热量:6.50;</td><td>热量浪费比率:1.34%</td></tr><tr><td>射速优先(level 7):</td><td>射速（发/秒）:13.32;</td><td>浪费热量:7.00;</td><td>热量浪费比率:1.30%</td></tr><tr><td>射速优先(level 8):</td><td>射速（发/秒）:14.69;</td><td>浪费热量:7.50;</td><td>热量浪费比率:1.26%</td></tr><tr><td>射速优先(level 9):</td><td>射速（发/秒）:16.05;</td><td>浪费热量:8.00;</td><td>热量浪费比率:1.23%</td></tr><tr><td>射速优先(level 10):</td><td>射速（发/秒）:17.80;</td><td>浪费热量:8.00;</td><td>热量浪费比率:1.11%</td></tr></table>

# 输入尿干时间(s):5
<table><tr><td>爆发优先(level 1):</td><td>射速（发/秒）:4.98;</td><td>浪费热量:1.00;</td><td>热量浪费比率:0.40%</td></tr><tr><td>爆发优先(level 2):</td><td>射速（发/秒）:6.47;</td><td>浪费热量:1.50;</td><td>热量浪费比率:0.46%</td></tr><tr><td>爆发优先(level 3):</td><td>射速（发/秒）:7.96;</td><td>浪费热量:2.00;</td><td>热量浪费比率:0.50%</td></tr><tr><td>爆发优先(level 4):</td><td>射速（发/秒）:9.45;</td><td>浪费热量:2.50;</td><td>热量浪费比率:0.53%</td></tr><tr><td>爆发优先(level 5):</td><td>射速（发/秒）:10.94;</td><td>浪费热量:3.00;</td><td>热量浪费比率:0.55%</td></tr><tr><td>爆发优先(level 6):</td><td>射速（发/秒）:12.43;</td><td>浪费热量:3.50;</td><td>热量浪费比率:0.56%</td></tr><tr><td>爆发优先(level 7):</td><td>射速（发/秒）:13.92;</td><td>浪费热量:4.00;</td><td>热量浪费比率:0.57%</td></tr><tr><td>爆发优先(level 8):</td><td>射速（发/秒）:15.41;</td><td>浪费热量:4.50;</td><td>热量浪费比率:0.58%</td></tr><tr><td>爆发优先(level 9):</td><td>射速（发/秒）:16.90;</td><td>浪费热量:5.00;</td><td>热量浪费比率:0.59%</td></tr><tr><td>爆发优先(level 10):</td><td>射速（发/秒）:18.88;</td><td>浪费热量:6.00;</td><td>热量浪费比率:0.63%</td></tr><tr><td>射速优先(level 1):</td><td>射速（发/秒）:4.92;</td><td>浪费热量:4.00;</td><td>热量浪费比率:1.60%</td></tr><tr><td>射速优先(level 2):</td><td>射速（发/秒）:6.11;</td><td>浪费热量:4.50;</td><td>热量浪费比率:1.45%</td></tr><tr><td>射速优先(level 3):</td><td>射速（发/秒）:7.30;</td><td>浪费热量:5.00;</td><td>热量浪费比率:1.35%</td></tr><tr><td>射速优先(level 4):</td><td>射速（发/秒）:8.49;</td><td>浪费热量:5.50;</td><td>热量浪费比率:1.28%</td></tr><tr><td>射速优先(level 5):</td><td>射速（发/秒）:9.68;</td><td>浪费热量:6.00;</td><td>热量浪费比率:1.22%</td></tr><tr><td>射速优先(level 6):</td><td>射速（发/秒）:10.87;</td><td>浪费热量:6.50;</td><td>热量浪费比率:1.18%</td></tr><tr><td>射速优先(level 7):</td><td>射速（发/秒）:12.06;</td><td>浪费热量:7.00;</td><td>热量浪费比率:1.15%</td></tr><tr><td>射速优先(level 8):</td><td>射速（发/秒）:13.25;</td><td>浪费热量:7.50;</td><td>热量浪费比率:1.12%</td></tr><tr><td>射速优先(level 9):</td><td>射速（发/秒）:14.44;</td><td>浪费热量:8.00;</td><td>热量浪费比率:1.10%</td></tr><tr><td>射速优先(level 10):</td><td>射速（发/秒）:15.84;</td><td>浪费热量:8.00;</td><td>热量浪费比率:1.00%</td></tr></table>
可以看见，无论在射击持续时间是多少，最终浪费（剩余）的热量都不会超过d（10）。



- example
```cpp
float a = (float)(ext_robot_status.shooter_barrel_cooling_value);
float m = (float)(ext_robot_status.shooter_barrel_heat_limit - ext_power_heat_data.shooter_17mm_1_barrel_heat);
float d = 10.0f;                
if(shoot_time == 0){
    /*方案二：根据热量上限和冷却决定射击策略，计算得当射击时间为m（热量上限）+1*a（冷却速率）时基本可以抹除冷却优先和爆发优的差距，即两者各级对应射速相近
             当k增大时，差距射击频率差距主要体现在低等级（爆发高，冷却低），等级越高影响越小。爆发模式下各等级射频更加均匀且持续时间更长，
             冷却模式正好相反，低等级射频低，高等级射频高且持续时间短，可灵活选择m+k*a*/
    ShootTime = (m + 2 * a) * 10;
    fn_Uint16Limit(&ShootTime,ShootTimeLower,ShootTimeUpper);
    //分级射速
    if(m < 100){
        shoot_speed = (10 * m - a - 3 * d) / (d * (ShootTime / 100.0f)) + a / d;
    }
    else{
        shoot_speed = (10 * m - a - 5 * d) / (d * (ShootTime / 100.0f)) + a / d;
    }
}
else if(0 < shoot_time && shoot_time < ShootTime){
    trigger_motor2006_data[0].target_speed = shoot_speed * 2 * PI / 7;
    fn_Fp32Limit(&trigger_motor2006_data[0].target_speed,0.0f,18.0f);
}
else{
    trigger_motor2006_data[0].target_speed = (a / d) * 2 * PI / 7;
    fn_Fp32Limit(&trigger_motor2006_data[0].target_speed,0.0f,18.0f);
}
if(shoot_time < ShootTime){
    shoot_time++;
}
```