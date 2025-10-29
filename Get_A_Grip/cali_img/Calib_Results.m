% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 907.016155766797283 ; 912.211105886656696 ];

%-- Principal point:
cc = [ 660.156166957534424 ; 384.191277635435142 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.167298999562626 ; -0.354312559662885 ; -0.003249389155066 ; 0.002937533008856 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 4.097054558682331 ; 3.286776820657313 ];

%-- Principal point uncertainty:
cc_error = [ 2.084007043351921 ; 5.616145887179769 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.006253308399002 ; 0.022827260623018 ; 0.000766049063281 ; 0.000952947815261 ; 0.000000000000000 ];

%-- Image size:
nx = 1280;
ny = 720;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 12;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 1.692043e+00 ; 1.670171e+00 ; -8.083586e-01 ];
Tc_1  = [ -1.281661e+02 ; -6.875262e+01 ; 3.367773e+02 ];
omc_error_1 = [ 3.510576e-03 ; 3.686594e-03 ; 4.509513e-03 ];
Tc_error_1  = [ 7.759890e-01 ; 1.981805e+00 ; 1.502106e+00 ];

%-- Image #2:
omc_2 = [ 1.869444e+00 ; 1.404102e+00 ; -6.786008e-01 ];
Tc_2  = [ -1.323169e+02 ; -5.718952e+01 ; 3.209574e+02 ];
omc_error_2 = [ 3.973152e-03 ; 3.277929e-03 ; 4.034436e-03 ];
Tc_error_2  = [ 7.403077e-01 ; 1.896636e+00 ; 1.451722e+00 ];

%-- Image #3:
omc_3 = [ 2.061985e+00 ; 9.991165e-01 ; -4.768273e-01 ];
Tc_3  = [ -1.214477e+02 ; -3.845686e+01 ; 2.957936e+02 ];
omc_error_3 = [ 4.531807e-03 ; 2.589584e-03 ; 3.393350e-03 ];
Tc_error_3  = [ 6.819589e-01 ; 1.763513e+00 ; 1.356847e+00 ];

%-- Image #4:
omc_4 = [ 1.995059e+00 ; 1.163630e+00 ; -5.570099e-01 ];
Tc_4  = [ -1.334066e+02 ; -3.074035e+01 ; 2.864620e+02 ];
omc_error_4 = [ 4.318277e-03 ; 2.844461e-03 ; 3.644027e-03 ];
Tc_error_4  = [ 6.596826e-01 ; 1.720691e+00 ; 1.317772e+00 ];

%-- Image #5:
omc_5 = [ 1.786472e+00 ; 1.541591e+00 ; -7.432916e-01 ];
Tc_5  = [ -1.336191e+02 ; -6.974532e+01 ; 3.360533e+02 ];
omc_error_5 = [ 3.739837e-03 ; 3.494344e-03 ; 4.276708e-03 ];
Tc_error_5  = [ 7.760016e-01 ; 1.973485e+00 ; 1.514212e+00 ];

%-- Image #6:
omc_6 = [ 1.426773e+00 ; 1.998484e+00 ; -9.552016e-01 ];
Tc_6  = [ -8.986091e+01 ; -1.203050e+02 ; 3.948883e+02 ];
omc_error_6 = [ 2.861542e-03 ; 4.077715e-03 ; 5.116694e-03 ];
Tc_error_6  = [ 9.222193e-01 ; 2.262350e+00 ; 1.728401e+00 ];

%-- Image #7:
omc_7 = [ 1.176293e+00 ; 2.214356e+00 ; -1.067929e+00 ];
Tc_7  = [ -6.910591e+01 ; -1.274167e+02 ; 4.093889e+02 ];
omc_error_7 = [ 2.392535e-03 ; 4.231501e-03 ; 5.590422e-03 ];
Tc_error_7  = [ 9.542567e-01 ; 2.340565e+00 ; 1.772179e+00 ];

%-- Image #8:
omc_8 = [ 1.512927e+00 ; 2.171288e+00 ; -7.701767e-01 ];
Tc_8  = [ -1.299856e+02 ; -1.163431e+02 ; 4.290440e+02 ];
omc_error_8 = [ 2.227048e-03 ; 3.562500e-03 ; 5.014635e-03 ];
Tc_error_8  = [ 9.922163e-01 ; 2.462514e+00 ; 2.008954e+00 ];

%-- Image #9:
omc_9 = [ 1.937326e+00 ; 1.809771e+00 ; -5.969334e-01 ];
Tc_9  = [ -1.279898e+02 ; -7.979873e+01 ; 3.875539e+02 ];
omc_error_9 = [ 2.662643e-03 ; 2.960491e-03 ; 4.519376e-03 ];
Tc_error_9  = [ 8.911862e-01 ; 2.258050e+00 ; 1.811934e+00 ];

%-- Image #10:
omc_10 = [ 2.311962e+00 ; 1.210312e+00 ; -3.331786e-01 ];
Tc_10  = [ -1.529835e+02 ; -4.540269e+01 ; 3.560647e+02 ];
omc_error_10 = [ 3.354979e-03 ; 2.176955e-03 ; 3.692231e-03 ];
Tc_error_10  = [ 8.226692e-01 ; 2.118557e+00 ; 1.665938e+00 ];

%-- Image #11:
omc_11 = [ 2.120487e+00 ; 2.135793e+00 ; -2.672210e-01 ];
Tc_11  = [ -1.172722e+02 ; -8.862551e+01 ; 3.383746e+02 ];
omc_error_11 = [ 1.702931e-03 ; 2.025902e-03 ; 3.949153e-03 ];
Tc_error_11  = [ 7.807829e-01 ; 1.968662e+00 ; 1.550693e+00 ];

%-- Image #12:
omc_12 = [ 1.916562e+00 ; 1.836055e+00 ; -5.563903e-01 ];
Tc_12  = [ -1.423543e+02 ; -6.983990e+01 ; 3.323331e+02 ];
omc_error_12 = [ 2.653201e-03 ; 2.926516e-03 ; 4.351861e-03 ];
Tc_error_12  = [ 7.639851e-01 ; 1.938932e+00 ; 1.591569e+00 ];

