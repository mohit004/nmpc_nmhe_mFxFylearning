# nmpc_nmhe_mFxFylearning
ROS packages for Nonlinear model predictive controller (NMPC) and nonlinear moving horizon estimator (NMHE).The resulting (high-level) NMPC tracks the commanded position trajectory while expecting the learned values for $m$, $F_{xDist}$, and $F_{yDist}$. Whereas, NMHE learn these parameters and feed them to NMPC. 

**Usage:**
1. roslaunch nmpc_pc_mfxfylearning_pkg nmpc_pc_mfxfylearning.launch
2. roslaunch nmhe_mfxfylearning_pkg nmhe_mfxfylearning.launch

Moreover, the following package can be used to command the reference trajectory:\
[trajectories](https://github.com/mohit004/trajectories): A ROS package to generate reference trajectories.

These NMPC and NMHE packages are utilized in the following works. Please don't forget to consider citing them if you use these code in your work.
**Plain Text:**
```
Mehndiratta M., Kayacan E., Patel S., Kayacan E., Chowdhary G. (2019) Learning-Based Fast Nonlinear Model Predictive Control for Custom-Made 3D Printed Ground and Aerial Robots. In: Raković S., Levine W. (eds) Handbook of Model Predictive Control. Control Engineering. Birkhäuser, Cham. https://doi.org/10.1007/978-3-319-77489-3_24
```
**Bibtex:**
```
@Inbook{mehndiratta2019a,
  author="Mehndiratta, Mohit
  and Kayacan, Erkan
  and Patel, Siddharth
  and Kayacan, Erdal
  and Chowdhary, Girish",
  editor="Rakovi{\'{c}}, Sa{\v{s}}a V. and Levine, William S.",
  title="Learning-Based Fast Nonlinear Model Predictive Control for Custom-Made 3D Printed Ground and Aerial Robots",
  bookTitle="Handbook of Model Predictive Control",
  year="2019",
  publisher="Springer International Publishing",
  address="Cham",
  pages="581--605",
  isbn="978-3-319-77489-3",
  doi="10.1007/978-3-319-77489-3_24",
}
```

**Plain Text:**
```
Mehndiratta, M., Kayacan, E. A constrained instantaneous learning approach for aerial package delivery robots: onboard implementation and experimental results. Auton Robot 43, 2209–2228 (2019). https://doi.org/10.1007/s10514-019-09875-y
```
**Bibtex:**
```
@Article{Mehndiratta2019,
  author="Mehndiratta, Mohit
  and Kayacan, Erdal",
  title="A constrained instantaneous learning approach for aerial package delivery robots: onboard implementation and experimental results",
  journal="Autonomous Robots",
  year="2019",
  month="Dec",
  day="01",
  volume="43",
  number="8",
  pages="2209--2228",
  issn="1573-7527",
  doi="10.1007/s10514-019-09875-y",
}
```
