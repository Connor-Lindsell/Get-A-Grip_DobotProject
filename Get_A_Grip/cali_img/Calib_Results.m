% Intrinsic and Extrinsic Camera Parameters
%
% This script file can be directly executed under Matlab to recover the camera intrinsic and extrinsic parameters.
% IMPORTANT: This file contains neither the structure of the calibration objects nor the image coordinates of the calibration points.
%            All those complementary variables are saved in the complete matlab data file Calib_Results.mat.
% For more information regarding the calibration model visit http://www.vision.caltech.edu/bouguetj/calib_doc/


%-- Focal length:
fc = [ 692.609244167284487 ; 690.065254803508310 ];

%-- Principal point:
cc = [ 284.905512041129725 ; 244.626846325448014 ];

%-- Skew coefficient:
alpha_c = 0.000000000000000;

%-- Distortion coefficients:
kc = [ 0.106697069970503 ; -0.250539104052457 ; 0.000104056976962 ; -0.000248328870229 ; 0.000000000000000 ];

%-- Focal length uncertainty:
fc_error = [ 2.055485917490698 ; 2.032757178751607 ];

%-- Principal point uncertainty:
cc_error = [ 3.421390156376911 ; 2.560301489386759 ];

%-- Skew coefficient uncertainty:
alpha_c_error = 0.000000000000000;

%-- Distortion coefficients uncertainty:
kc_error = [ 0.014402463825879 ; 0.075062905103806 ; 0.001533228057144 ; 0.002181729763416 ; 0.000000000000000 ];

%-- Image size:
nx = 1280;
ny = 720;


%-- Various other variables (may be ignored if you do not use the Matlab Calibration Toolbox):
%-- Those variables are used to control which intrinsic parameters should be optimized

n_ima = 9;						% Number of calibration images
est_fc = [ 1 ; 1 ];					% Estimation indicator of the two focal variables
est_aspect_ratio = 1;				% Estimation indicator of the aspect ratio fc(2)/fc(1)
center_optim = 1;					% Estimation indicator of the principal point
est_alpha = 0;						% Estimation indicator of the skew coefficient
est_dist = [ 1 ; 1 ; 1 ; 1 ; 0 ];	% Estimation indicator of the distortion coefficients


%-- Extrinsic parameters:
%-- The rotation (omc_kk) and the translation (Tc_kk) vectors for every calibration image and their uncertainties

%-- Image #1:
omc_1 = [ 2.077801e+00 ; 2.168376e+00 ; -1.539235e-01 ];
Tc_1  = [ -2.369472e+02 ; -2.940662e+02 ; 1.301212e+03 ];
omc_error_1 = [ 3.138796e-03 ; 3.614805e-03 ; 7.201970e-03 ];
Tc_error_1  = [ 6.483845e+00 ; 4.812721e+00 ; 4.330625e+00 ];

%-- Image #2:
omc_2 = [ 2.176266e+00 ; 2.248133e+00 ; 1.742594e-01 ];
Tc_2  = [ -2.948064e+02 ; -1.869084e+02 ; 1.457222e+03 ];
omc_error_2 = [ 4.097148e-03 ; 3.572207e-03 ; 8.477775e-03 ];
Tc_error_2  = [ 7.250341e+00 ; 5.461227e+00 ; 5.094703e+00 ];

%-- Image #3:
omc_3 = [ -1.954025e+00 ; -2.133859e+00 ; 4.092194e-01 ];
Tc_3  = [ -2.118014e+02 ; -2.940693e+02 ; 1.445334e+03 ];
omc_error_3 = [ 3.578674e-03 ; 3.413905e-03 ; 6.598522e-03 ];
Tc_error_3  = [ 7.167892e+00 ; 5.329609e+00 ; 4.471795e+00 ];

%-- Image #4:
omc_4 = [ 1.855033e+00 ; 1.802657e+00 ; -6.191439e-01 ];
Tc_4  = [ -3.075635e+02 ; -1.475771e+02 ; 1.585499e+03 ];
omc_error_4 = [ 2.676663e-03 ; 3.878552e-03 ; 6.048161e-03 ];
Tc_error_4  = [ 7.825372e+00 ; 5.875747e+00 ; 4.776200e+00 ];

%-- Image #5:
omc_5 = [ 1.981797e+00 ; 1.702323e+00 ; 2.159431e-01 ];
Tc_5  = [ -1.826871e+02 ; -2.704443e+02 ; 1.419359e+03 ];
omc_error_5 = [ 3.690796e-03 ; 3.413256e-03 ; 6.131393e-03 ];
Tc_error_5  = [ 7.072359e+00 ; 5.239016e+00 ; 4.886046e+00 ];

%-- Image #6:
omc_6 = [ -1.841883e+00 ; -1.904901e+00 ; -4.662844e-01 ];
Tc_6  = [ -2.326053e+02 ; -2.861068e+02 ; 1.219539e+03 ];
omc_error_6 = [ 2.424699e-03 ; 3.922015e-03 ; 6.208793e-03 ];
Tc_error_6  = [ 6.116542e+00 ; 4.548843e+00 ; 4.432200e+00 ];

%-- Image #7:
omc_7 = [ 1.631440e+00 ; 1.512811e+00 ; -1.370501e-02 ];
Tc_7  = [ -3.573102e+02 ; -2.218870e+02 ; 1.563038e+03 ];
omc_error_7 = [ 3.184385e-03 ; 3.940139e-03 ; 5.061319e-03 ];
Tc_error_7  = [ 7.766528e+00 ; 5.827518e+00 ; 5.439275e+00 ];

%-- Image #8:
omc_8 = [ 1.778401e+00 ; 2.189561e+00 ; -1.195770e+00 ];
Tc_8  = [ -2.451156e+02 ; -3.003783e+02 ; 1.728525e+03 ];
omc_error_8 = [ 2.170645e-03 ; 4.630575e-03 ; 6.904718e-03 ];
Tc_error_8  = [ 8.615111e+00 ; 6.444542e+00 ; 4.716188e+00 ];

%-- Image #9:
omc_9 = [ -2.594884e+00 ; -3.943789e-01 ; 1.335029e+00 ];
Tc_9  = [ -3.161397e+01 ; 3.907895e+01 ; 1.802690e+03 ];
omc_error_9 = [ 4.772436e-03 ; 2.061031e-03 ; 6.513998e-03 ];
Tc_error_9  = [ 8.891151e+00 ; 6.641597e+00 ; 4.110394e+00 ];

