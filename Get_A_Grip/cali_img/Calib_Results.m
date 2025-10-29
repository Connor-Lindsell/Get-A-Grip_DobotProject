% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 907.024788462619540 ; 912.217491903231803 ];

%-- Principal point:
cc = [ 660.147902420294031 ; 384.183931450628052 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.167264104498471 ; -0.354144752349849 ; -0.003249276293750 ; 0.002932691469087 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 4.097908070028614 ; 3.287263150948830 ];

%-- Principal point uncertainty:
cc_error = [ 2.084486903039132 ; 5.617536324266790 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.006253462739538 ; 0.022828003888234 ; 0.000766206369618 ; 0.000953202071066 ; 0.000000000000000 ];

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
omc_1 = [ 1.692038e+00 ; 1.670174e+00 ; -8.083730e-01 ];
Tc_1  = [ -1.281629e+02 ; -6.875004e+01 ; 3.367809e+02 ];
omc_error_1 = [ 3.511436e-03 ; 3.687533e-03 ; 4.510436e-03 ];
Tc_error_1  = [ 7.761708e-01 ; 1.982304e+00 ; 1.502424e+00 ];

%-- Image #2:
omc_2 = [ 1.869439e+00 ; 1.404105e+00 ; -6.786138e-01 ];
Tc_2  = [ -1.323139e+02 ; -5.718720e+01 ; 3.209612e+02 ];
omc_error_2 = [ 3.974126e-03 ; 3.278757e-03 ; 4.035243e-03 ];
Tc_error_2  = [ 7.404817e-01 ; 1.897117e+00 ; 1.452028e+00 ];

%-- Image #3:
omc_3 = [ 2.061979e+00 ; 9.991201e-01 ; -4.768398e-01 ];
Tc_3  = [ -1.214451e+02 ; -3.845465e+01 ; 2.957970e+02 ];
omc_error_3 = [ 4.532923e-03 ; 2.590224e-03 ; 3.394016e-03 ];
Tc_error_3  = [ 6.821188e-01 ; 1.763959e+00 ; 1.357129e+00 ];

%-- Image #4:
omc_4 = [ 1.995053e+00 ; 1.163633e+00 ; -5.570226e-01 ];
Tc_4  = [ -1.334040e+02 ; -3.073816e+01 ; 2.864656e+02 ];
omc_error_4 = [ 4.319337e-03 ; 2.845170e-03 ; 3.644737e-03 ];
Tc_error_4  = [ 6.598381e-01 ; 1.721127e+00 ; 1.318045e+00 ];

%-- Image #5:
omc_5 = [ 1.786466e+00 ; 1.541594e+00 ; -7.433055e-01 ];
Tc_5  = [ -1.336159e+02 ; -6.974285e+01 ; 3.360572e+02 ];
omc_error_5 = [ 3.740754e-03 ; 3.495234e-03 ; 4.277574e-03 ];
Tc_error_5  = [ 7.761841e-01 ; 1.973985e+00 ; 1.514531e+00 ];

%-- Image #6:
omc_6 = [ 1.426767e+00 ; 1.998481e+00 ; -9.552162e-01 ];
Tc_6  = [ -8.985775e+01 ; -1.203016e+02 ; 3.948906e+02 ];
omc_error_6 = [ 2.862245e-03 ; 4.078774e-03 ; 5.117780e-03 ];
Tc_error_6  = [ 9.224296e-01 ; 2.262915e+00 ; 1.728771e+00 ];

%-- Image #7:
omc_7 = [ 1.176288e+00 ; 2.214358e+00 ; -1.067943e+00 ];
Tc_7  = [ -6.910222e+01 ; -1.274136e+02 ; 4.093919e+02 ];
omc_error_7 = [ 2.393097e-03 ; 4.232622e-03 ; 5.591651e-03 ];
Tc_error_7  = [ 9.544765e-01 ; 2.341155e+00 ; 1.772565e+00 ];

%-- Image #8:
omc_8 = [ 1.512924e+00 ; 2.171293e+00 ; -7.701893e-01 ];
Tc_8  = [ -1.299816e+02 ; -1.163400e+02 ; 4.290485e+02 ];
omc_error_8 = [ 2.227550e-03 ; 3.563384e-03 ; 5.015638e-03 ];
Tc_error_8  = [ 9.924472e-01 ; 2.463139e+00 ; 2.009383e+00 ];

%-- Image #9:
omc_9 = [ 1.937324e+00 ; 1.809775e+00 ; -5.969470e-01 ];
Tc_9  = [ -1.279862e+02 ; -7.979591e+01 ; 3.875581e+02 ];
omc_error_9 = [ 2.663249e-03 ; 2.961197e-03 ; 4.520247e-03 ];
Tc_error_9  = [ 8.913958e-01 ; 2.258622e+00 ; 1.812318e+00 ];

%-- Image #10:
omc_10 = [ 2.311960e+00 ; 1.210315e+00 ; -3.331907e-01 ];
Tc_10  = [ -1.529802e+02 ; -4.540006e+01 ; 3.560689e+02 ];
omc_error_10 = [ 3.355759e-03 ; 2.177425e-03 ; 3.692884e-03 ];
Tc_error_10  = [ 8.228623e-01 ; 2.119094e+00 ; 1.666278e+00 ];

%-- Image #11:
omc_11 = [ 2.120491e+00 ; 2.135800e+00 ; -2.672323e-01 ];
Tc_11  = [ -1.172690e+02 ; -8.862304e+01 ; 3.383783e+02 ];
omc_error_11 = [ 1.703025e-03 ; 2.026089e-03 ; 3.949575e-03 ];
Tc_error_11  = [ 7.809660e-01 ; 1.969162e+00 ; 1.551000e+00 ];

%-- Image #12:
omc_12 = [ 1.916562e+00 ; 1.836061e+00 ; -5.564033e-01 ];
Tc_12  = [ -1.423512e+02 ; -6.983755e+01 ; 3.323375e+02 ];
omc_error_12 = [ 2.653798e-03 ; 2.927194e-03 ; 4.352643e-03 ];
Tc_error_12  = [ 7.641664e-01 ; 1.939423e+00 ; 1.591902e+00 ];

