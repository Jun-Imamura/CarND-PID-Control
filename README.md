# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program, implementation of PID Contoller algorithm.

Please refer [project introduction](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562) page for further information.

---

## Summary
#### Effect analysis of PID components

###### P Controller
Firstly, I tried only P-controller implementation. This results in fractuated result when the vehicle goes into curved road.

|   |   Kp   |   Ki   |   Kd   |
|:--|--------|--------|--------|
|val|   0.135|     0.0|     0.0|

[result video](./output/P_Controller.mp4)

###### PD Controller
Next, I tried PD-controller implemetation. This result in much stable result, but CTE had some drift.

|   |   Kp   |   Ki   |   Kd   |
|:--|--------|--------|--------|
|val|   0.135|     0.0|     3.0|

[result video](./output/PD_Controller.mp4)

###### PID Controller
Finally, I tried PID-controller implemetation. In this parameter set, I could reduce the CTE drift.

|   |   Kp   |   Ki   |   Kd   |
|:--|--------|--------|--------|
|val|   0.135|  0.0002|     3.0|

[result video](./output/PID_Controller.mp4)

#### How the Final HyperParameters Chosen?
###### Twiddle
I applied twiddling which is explained in the lecture video. In order to do it for this assignment, we firstly need to set initial value which won't go out of the course.

Following code is the pseudo code of twiddling.

```python
while sum(dp) > 0.000001:
  for i in range(3):
      p[i] = dp[i]
      err = run(p)
    if err < besterr:
        besterr = err
        dp[i] *= 1.1
    else:
      p[i] -= 2*dp[i]
      if err < besterr:
        dp[i]*=1.1
      else:
        p[i] += dp[i]
        dp[i] *= 0.9
```

###### Inital value
As a initial parameter, we set values as follows:

|   |   Kp   |   Ki   |   Kd   |
|:--|--------|--------|--------|
|val|     0.1|   0.001|    10.0|

###### Parameters for Twiddling

```cpp
  twiddle_best_error = 100000000000; //init value (enough large)
  n_eval_twiddle_step = 10000;       //# of frames per each iteration
  n_eval_twiddle_loop = 100;         //# of loop
  dp = {0.1*Kp, 0.1*Ki, 0.1*Kd};     //initital differential value
```

###### Result
Below is the final parameter set.

|   |   Kp   |   Ki   |   Kd   |
|:--|--------|--------|--------|
|val|0.214795|0.00187485|8.02718|

This time, I set only total sum of CTE as the error function parameter. This makes the output video's steering strong.

[result video](./output/After_Twiddle.mp4)

You can also see how the parameter changed during twiddling [here](./output/twiddle.txt).