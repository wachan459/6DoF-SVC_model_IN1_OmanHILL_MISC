# 6DoF-SVC_model_IN1_OmanHILL_MISC
6DoF-SVC model for predicting motion sickness symptoms, with MISC as the output.

## Description
This is a test program for a motion sickness model that estimates symptom progression of motion sickness proposed in references [1][2]. This model estimates symptom progression of motion sickness by Motion Illness Symptoms Classification (MISC) based on the input of head-centered linear acceleration (GIF) f (=g+a) and angular velocity omega.The MISC is an 11-point symptom-based scale ranging from 0 (no symptoms) to 10 (vomiting). Scores of 1–5 indicate increasing severity of discomfort and symptoms without nausea, while scores of 6–8 indicate increasing severity of nausea, 9 indicates retching, and 10 indicates vomiting [3].

It is based on the Subjective Vertical Conflict (SVC) theory (Bles & Bos, 1998), which can be regarded as a refinement of the well-known Sensory Conflict Theory and Neural Mismatch Theory. The SVC model was subsequently extended to six-degree-of-freedom (6DoF) head motion [4]. While the conventional 6DoF-SVC model predicts motion sickness incidence (MSI), defined as the percentage of people expected to vomit, the present model predicts motion sickness severity using the Motion Illness Symptoms Classification (MISC) scale. To achieve this, the model incorporates the final-path dynamics referred to as "OmanHILL." Reference [2] compares several formulations of the final-path dynamics, and the present implementation adopts the most promising formulation identified in that study.

The program computes MSI in two main steps:

1) Computation of the initial state vector x(0):
Assuming the human body remains in a constant posture (i.e., the same input persists over time), the state vector x is iteratively computed until it converges. Any arbitrary initial value for x can be used for this step, as convergence is guaranteed (see [4]).

2) Main simulation:
Using the converged x from step 1) as the initial state (with MSI reset to zero), the program proceeds to compute the state vector x and MISC sequentially in response to the given input u.

The step 1) leverages a key property of the 6DoF-SVC (IN1) model described in [3]: under stationary input conditions, the internal states including perceived motion, converge to "appropriate" values. If the correct initial state x(0) is already known, step 1) may be skipped.

Note that step (1) is particularly useful when applying the 6DoF-SVC model to real-world IMU data. For example, it is effective when the IMU is not aligned with the intended coordinate system (e.g., due to tilted sensor mounting).

## Environment
This model has been tested using **MATLAB R2025b**.  

## References
[1] Inoue, S., Dang, V.T., Liu, H. et al. Construction of a computational model of individual progression of motion sickness symptoms based on subjective vertical conflict theory. Exp Brain Res 243, 108 (2025). https://doi.org/10.1007/s00221-025-07052-5

[2] V. T. Dang, M. Bourseul and T. Wada, "Prediction of Individual Motion Sickness of Bus Passengers Using a 6DoF-SVC Model Driven by Handheld Tablet Motion During Visual Tasks," in IEEE Transactions on Intelligent  Transportation Systems, vol. 27, no. 10, pp. 13461-13472, Oct. 2026, doi: 10.1109/TITS.2026.3710322.

[3] Bos, J. E., MacKinnon, S. N., & Patterson, A. (2005). Motion sickness symptoms in a ship motion simulator: effects of inside, outside, and no view. Aviation, Space, and Environmental Medicine, 76(12), 1111–1118.

[4] Wada, T., & Bos, J. E. (2025). Theoretical Considerations on Models of Vestibular Self-motion Perception as Inherent in Computational Frameworks of Motion Sickness. Biological Cybernetics.

## How to cite
If you use this model in your work, please cite:
- Inoue, S., Dang, V.T., Liu, H. et al. Construction of a computational model of individual progression of motion sickness symptoms based on subjective vertical conflict theory. Exp Brain Res 243, 108 (2025). https://doi.org/10.1007/s00221-025-07052-5
- V. T. Dang, M. Bourseul and T. Wada, "Prediction of Individual Motion Sickness of Bus Passengers Using a 6DoF-SVC Model Driven by Handheld Tablet Motion During Visual Tasks," in IEEE Transactions on Intelligent  Transportation Systems, vol. 27, no. 10, pp. 13461-13472, Oct. 2026, doi: 10.1109/TITS.2026.3710322.
- 
## License
This project is licensed under the MIT License.  
See the `LICENSE` file for details.
