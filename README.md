# optctrlmuscle
Optimal control approach to solving the muscle redundancy problem. Code expanded
on from the original SimTK project by Freidl De Groote, B.J. Fregly, Antoine
Falisse, and Maarten Afschrift (located here: https://simtk.org/projects/optcntrlmuscle.) 
Additional software required as described in the included Manual. For a more detailed
description of the general problem being solved, please refer to the associated paper:

[1] F. De Groote, A. L. Kinney, A. V. Rao, and B.J. Fregly, "Evaluation of Direct Collocation 
		Optimal Control Problem Formulations for Solving the Muscle Redundancy Problem," Annals 
		of Biomedical Engineering, 2016, DOI: 10.1007/s10439-016-1591-9

Branches
========

# master
Authors: Chris Dembia, Nick Bianco

Contains the base code from the original SimTK project and any relevant updates or 
improvements that benefit all subprojects. Subprojects exist as branches off of the
master branch (e.g. ankle_clutched_spring, soft_exosuit_design).

# soft_exosuit_design
Authors: Nick Bianco, Rachel Troutman

References:

[2] B.T. Quinlivan et al., "Assistance magnitude versus metabolic cost reductions for a
		tethered multiarticular soft exosuit," Science Robotics, 2017, Vol. 2, eaah4416

[3] R.C. Browning et al., "The Effects of Adding Mass to the Legs on the Energetics and 
		Biomechanics of Walking," Medicine & Science in Sports & Exercise, 2007
		DOI: 10.1249/mss.0b013e31802b3562

[4] A.E. Minetti and R. McN. Alexander, "A Theory of Metabolic Costs for Bipedal Gaits,"
		Journal of Theoretical Biology, 1997, Vol. 186, pgs. 467-476

All examples within the Examples/SoftExosuitDesign subdirectory were created for the 
Stanford course ME485: Modeling and Simulation of Human Movement. A webpage containing
results and analysis of this project can be found on the OpenSim documentation site:
http://simtk-confluence.stanford.edu:8080/display/OpenSim/Simulation-based+soft+exosuit+design.

For this project, the following MATLAB m-files were either created or modified:

1) Problem files
These files call SolveMuscleRedundancy_lMtildeState to execute a particular predefined 
optimal control problem. If the examples included with the SimTK project run on your 
machine (i.e. OpenSim downloaded, GPOPS-II/Adigator installed, etc.), these examples should
run out of the box.

Quinlivan2017.m -- This problem attempts to replicate the study in [2] using the Gait2354
				   default model and data set that is packaged with the OpenSim distribution.
				   
HipAnkle.m -- This problem modifies the Quinlivan2017.m problem by optimizing for both the
			  tradeoff between assistive moments at the hip and ankle and for the device 
			  control signal.

HipKneeAnkle.m -- This problem is similar to the HipAnkle.m problem, where now the 
				  optimization is free to choose assistive knee flexion or extension
				  moments.
				 
HipExtHipAbd.m -- This problem is similar to the HipAnkle.m problem, except now the tradeoff
				  is optimized between hip extension and hip abduction assistance.
			
HipAnkleMass.m -- This problem is similar to the HipAnkle.m problem, except now assisting
				  a particular joint incurs a mass penalty associated with the relationship
				  presented in [3]. This problem is currently a work-in-progress.
				  
Each one of these files is contained within a subdirectory of the same name 
(i.e. Examples/SoftExosuitDesign/<ProblemName>). Folders within each of these directories
contain results obtained by using different integrated cost functions:

Exc_Act -- Excitations and activations squared. (Results from our project utilized
		   only this cost function, but we have included our other results as well.)

MinAlex -- Metabolic rate (Minetti and Alexander 1997 [4]).

Exc_Act_MinAlex -- Excitations and activations squared + metabolic rate

Note: all of the problems under the HipAnkleMass subdirectory include the device mass
penalty in the cost, even though the result folder names do not reflect this. 

Various plotting scripts exist in many of the subdirectories. These were used
to generate the plots of our project results.

3) SolveMuscleRedundancy_lMtildeState.m

This file serves as the "main" function for solving the muscle redundancy problem. Different
problem files call this function to execute a particular predefined optimal control problem.
The optimal control problems called by this file introduce normalized muscle fiber length as
a state variable for solving the implicit muscle dynamics formulation described in [1]. 
(Another similar file called SolveMuscleRedundancy_FtildeState solves the same problem, but
introduces normalized tendon force as a state variable instead, see [1]. No changes were made
to this file, as we only considered problems using normalized fiber length as a state.) 

The section labeled "PART II: OPTIMAL CONTROL PROBLEM FORMULATION", is where most code
modifications are located. Based on the choice of problem in section 1), the structure, bounds,
and initial guesses of the controls and parameters are modified. Data structures containing 
force and moment data from the Quinlivan et al. 2017 study [2] are created as necessary for 
each problem. All of this information is passed on to the continous and endpoint functions,
see section 3).

4) Continous and Endpoint Functions

In the subdirectory Optimization\lMtildeState, there are multiple folders containing m-files
that define the continous and endpoint functions needed by GPOPS-II to solve the optimal 
control problem. The continous function defines any path constraints, dynamics constraints,
or integrated objectives for a given optimal control problem. For our project, this is where
we modified the inverse dynamics moment tracking constraint (see the PATH CONSTRAINTS section
in each continous function file). The endpoint function defines any endpoints contraints 
or bounds, including initial or final state values or periodicity. For more details on 
continous and endpoint functions in general, please refer to the GPOPS-II instruction manual.