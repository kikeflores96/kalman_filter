Module EKF-SLAM -- BE
ISAE, 19-20 NOVEMBER 2020
By Joan Sola and Ellon Paiva Mendes

---------------------------------------------------------------------------
*** Support documents (find them on the LMS)
- This README.txt
- SLAM-course-BE.pdf
- SLAM-course-BE-slides.pdf
---------------------------------------------------------------------------


*** BE. Way to proceed

1. Get the code and prepare your Matlab workspace

- Get the slam2d_BE.zip from the LMS
- Unpack slam2d_BE.zip
- Launch Matlab
- Set Matlab path to slam2d_BE folder
- Launch slam2d_BE.m. See that SLAM is not complete: it only simulates a robot, but it does not do any SLAM.
- Dock the figure onto your workspace (click on the little curved arrow on the upper right corner).


2. (19/11/2020) Write the observation function observe.m

- Get inspired by invObserve.m.
- Use toFrame.m and scan.m as building blocks.
- Use the chain rule to build the Jacobian matrices.
- Test the function by running 'test_observe' on the Matlab command window
   -->  This function checks that all is correct: the function and the Jacobians.


3. (20/11/2020) Complete the Range-and-Bearing SLAM code

- Edit slam2d_BE.m
- Examine slam2d_BE code: You need to enter code wherever you find the string '<...>', and uncomment that line.
- Carefully read the HELP lines at the beginning of the file slam2d_BE.m. They contain important information for coding.

- HOW TO PROCEED:

 	Uncomment all SLAM code in slam2d_BE.m, in the section ESTIMATOR, which is the SLAM algorithm.
		Use only data provided by the simulator, provided by sim_* functions. See HELP section in slam2d_BE.m
		Follow numbered comments like '% 2. compute robot covariance'.
		Get inspired by the PDF document SLAM-course_BE.pdf

	Debug until no failure.

    You can always execute slam2d.m and check if your results are similar to ours.

	Make a copy of your slam2d_BE.m named slam2d_BE_solved.m. This file should contain your solution for the BE.

	Play with slam2d_BE.m (NOT the solved version you just created!), experimenting with different values of:
		Process noise, q;
		Measurement noise, s;
		Initial robot estimate;
		Initial robot uncertainty;
		Map size.


4. You should submit to the LMS:

		The directory slam2d_BE with the whole SLAM code;

		A text file answering the following questions:
			If we increase the control noise, what happens? And why?
			If we increase the measurement noise, what happens? And why?
			If we change the initial robot position, what happens? And why?
			If we increase the initial robot position uncertainty, what happens? And why?
			Why do we need to make sure that the innovation angle z(2) is centered around zero in section c6?
			If we make the variable 'mapsize' smaller, what happens? And why?
			Could you explain which parts of this code you should improve or modify if you were to use this code in a real robot, with real data?


	NOTE: make sure that slam2d_BE_solved.m contains your working solution for the BE! This is the file that will be evaluated in the correction.


Copyright 2014-2020 Joan Sola @ IRI-UPC-CSIC
Copyright 2017-2020 Ellon Paiva @ EasyMile
