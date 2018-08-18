function printLowPassFilterOutput()
% 	Function to print out the test outputs for the Low Pass Filter.
%
% 	Inputs:
%
%			none 	Test cases are hard coded.
%
% 	Outputs:
%
%			none 	Test cases are printed to screen.
%
% 	Griswald Brooks
% 	griswald.brooks@gmail.com
%

% Test inputs.
x_i = [0.447265, 0.677158, 0.490548, 0.896610, 0.948445, 0.748019, 0.050977, 0.457688, 0.448624, 0.810386];
y_i = [0.775992, 0.873013, 0.335330, 0.174261, 0.405399, 0.119830, 0.052253, 0.465377, 0.655518, 0.310589];
w_i = [0.991499, 0.539988, 0.447987, 0.544278, 0.601881, 0.976729, 0.402125, 0.419183, 0.074944, 0.571029];

% Pad inputs with history.
x_i = [x_i(1), x_i(1), x_i(1), x_i];
y_i = [y_i(1), y_i(1), y_i(1), y_i];
w_i = [w_i(1), w_i(1), w_i(1), w_i];

% Test coefficients.
% First order, 0.2 rad/sec (normalized cutoff frequency).
b_fo_arr = [0.24524,  0.24524];
a_fo_arr = [1.00000, -0.50953];
% Second order, 0.5 rad/sec (normalized cutoff frequency).
b_so_arr = [0.29289, 0.58579, 0.29289];
a_so_arr = [1.0000e+00, -1.3878e-16, 1.7157e-01];
% Third order, 0.7 rad/sec (normalized cutoff frequency).
b_to_arr = [0.37445, 1.12336, 1.12336, 0.37445];
a_to_arr = [1.00000, 1.16192, 0.69594, 0.13776];

% Get test outputs for uninitialized filter.
x_fo_ui_o = filter(b_fo_arr, a_fo_arr, x_i);
y_fo_ui_o = filter(b_fo_arr, a_fo_arr, y_i);
w_fo_ui_o = filter(b_fo_arr, a_fo_arr, w_i);

x_so_ui_o = filter(b_so_arr, a_so_arr, x_i);
y_so_ui_o = filter(b_so_arr, a_so_arr, y_i);
w_so_ui_o = filter(b_so_arr, a_so_arr, w_i);

x_to_ui_o = filter(b_to_arr, a_to_arr, x_i);
y_to_ui_o = filter(b_to_arr, a_to_arr, y_i);
w_to_ui_o = filter(b_to_arr, a_to_arr, w_i);

% Get test outputs for input initialized filter.
x_fo_ii_o = zeros(1,13);
y_fo_ii_o = zeros(1,13);
w_fo_ii_o = zeros(1,13);

x_so_ii_o = zeros(1,13);
y_so_ii_o = zeros(1,13);
w_so_ii_o = zeros(1,13);

x_to_ii_o = zeros(1,13);
y_to_ii_o = zeros(1,13);
w_to_ii_o = zeros(1,13);

for i = 1:10
	% Generate sample indices. Samples are padded so they use a valid history.
	n = i + 3;
	nm1 = n - 1;
	nm2 = n - 2;
	nm3 = n - 3;

	% Loop over samples.
	x_fo_ii_o(n) = b_fo_arr(1)*x_i(n) + b_fo_arr(2)*x_i(nm1) - a_fo_arr(2)*x_fo_ii_o(nm1);
	y_fo_ii_o(n) = b_fo_arr(1)*y_i(n) + b_fo_arr(2)*y_i(nm1) - a_fo_arr(2)*y_fo_ii_o(nm1);
	w_fo_ii_o(n) = b_fo_arr(1)*w_i(n) + b_fo_arr(2)*w_i(nm1) - a_fo_arr(2)*w_fo_ii_o(nm1);


	x_so_ii_o(n) = b_so_arr(1)*x_i(n) + b_so_arr(2)*x_i(nm1) + b_so_arr(3)*x_i(nm2) - a_so_arr(2)*x_so_ii_o(nm1) - a_so_arr(3)*x_so_ii_o(nm2);
	y_so_ii_o(n) = b_so_arr(1)*y_i(n) + b_so_arr(2)*y_i(nm1) + b_so_arr(3)*y_i(nm2) - a_so_arr(2)*y_so_ii_o(nm1) - a_so_arr(3)*y_so_ii_o(nm2);
	w_so_ii_o(n) = b_so_arr(1)*w_i(n) + b_so_arr(2)*w_i(nm1) + b_so_arr(3)*w_i(nm2) - a_so_arr(2)*w_so_ii_o(nm1) - a_so_arr(3)*w_so_ii_o(nm2);

	x_to_ii_o(n) = b_to_arr(1)*x_i(n) + b_to_arr(2)*x_i(nm1) + b_to_arr(3)*x_i(nm2) + b_to_arr(4)*x_i(nm3) - a_to_arr(2)*x_to_ii_o(nm1) - a_to_arr(3)*x_to_ii_o(nm2) - a_to_arr(4)*x_to_ii_o(nm3);
	y_to_ii_o(n) = b_to_arr(1)*y_i(n) + b_to_arr(2)*y_i(nm1) + b_to_arr(3)*y_i(nm2) + b_to_arr(4)*y_i(nm3) - a_to_arr(2)*y_to_ii_o(nm1) - a_to_arr(3)*y_to_ii_o(nm2) - a_to_arr(4)*y_to_ii_o(nm3);
	w_to_ii_o(n) = b_to_arr(1)*w_i(n) + b_to_arr(2)*w_i(nm1) + b_to_arr(3)*w_i(nm2) + b_to_arr(4)*w_i(nm3) - a_to_arr(2)*w_to_ii_o(nm1) - a_to_arr(3)*w_to_ii_o(nm2) - a_to_arr(4)*w_to_ii_o(nm3);
end

% Get test outputs for input/output initialized filter.
x_fo_io_o = x_i;
y_fo_io_o = y_i;
w_fo_io_o = w_i;

x_so_io_o = x_i;
y_so_io_o = y_i;
w_so_io_o = w_i;

x_to_io_o = x_i;
y_to_io_o = y_i;
w_to_io_o = w_i;

for i = 1:10
	% Generate sample indices. Samples are padded so they use a valid history.
	n = i + 3;
	nm1 = n - 1;
	nm2 = n - 2;
	nm3 = n - 3;

	% Loop over samples.
	x_fo_io_o(n) = b_fo_arr(1)*x_i(n) + b_fo_arr(2)*x_i(nm1) - a_fo_arr(2)*x_fo_io_o(nm1);
	y_fo_io_o(n) = b_fo_arr(1)*y_i(n) + b_fo_arr(2)*y_i(nm1) - a_fo_arr(2)*y_fo_io_o(nm1);
	w_fo_io_o(n) = b_fo_arr(1)*w_i(n) + b_fo_arr(2)*w_i(nm1) - a_fo_arr(2)*w_fo_io_o(nm1);

	x_so_io_o(n) = b_so_arr(1)*x_i(n) + b_so_arr(2)*x_i(nm1) + b_so_arr(3)*x_i(nm2) - a_so_arr(2)*x_so_io_o(nm1) - a_so_arr(3)*x_so_io_o(nm2);
	y_so_io_o(n) = b_so_arr(1)*y_i(n) + b_so_arr(2)*y_i(nm1) + b_so_arr(3)*y_i(nm2) - a_so_arr(2)*y_so_io_o(nm1) - a_so_arr(3)*y_so_io_o(nm2);
	w_so_io_o(n) = b_so_arr(1)*w_i(n) + b_so_arr(2)*w_i(nm1) + b_so_arr(3)*w_i(nm2) - a_so_arr(2)*w_so_io_o(nm1) - a_so_arr(3)*w_so_io_o(nm2);

	x_to_io_o(n) = b_to_arr(1)*x_i(n) + b_to_arr(2)*x_i(nm1) + b_to_arr(3)*x_i(nm2) + b_to_arr(4)*x_i(nm3) - a_to_arr(2)*x_to_io_o(nm1) - a_to_arr(3)*x_to_io_o(nm2) - a_to_arr(4)*x_to_io_o(nm3);
	y_to_io_o(n) = b_to_arr(1)*y_i(n) + b_to_arr(2)*y_i(nm1) + b_to_arr(3)*y_i(nm2) + b_to_arr(4)*y_i(nm3) - a_to_arr(2)*y_to_io_o(nm1) - a_to_arr(3)*y_to_io_o(nm2) - a_to_arr(4)*y_to_io_o(nm3);
	w_to_io_o(n) = b_to_arr(1)*w_i(n) + b_to_arr(2)*w_i(nm1) + b_to_arr(3)*w_i(nm2) + b_to_arr(4)*w_i(nm3) - a_to_arr(2)*w_to_io_o(nm1) - a_to_arr(3)*w_to_io_o(nm2) - a_to_arr(4)*w_to_io_o(nm3);
end

% Print test outputs.
disp('Output vectors for uninitialized filters:');
disp(['x_fo_ui_o = ', num2str(x_fo_ui_o(4:end))]);
disp(['y_fo_ui_o = ', num2str(y_fo_ui_o(4:end))]);
disp(['w_fo_ui_o = ', num2str(w_fo_ui_o(4:end))]);
disp('')
disp(['x_so_ui_o = ', num2str(x_so_ui_o(4:end))]);
disp(['y_so_ui_o = ', num2str(y_so_ui_o(4:end))]);
disp(['w_so_ui_o = ', num2str(w_so_ui_o(4:end))]);
disp('')
disp(['x_to_ui_o = ', num2str(x_to_ui_o(4:end))]);
disp(['y_to_ui_o = ', num2str(y_to_ui_o(4:end))]);
disp(['w_to_ui_o = ', num2str(w_to_ui_o(4:end))]);
disp('')
disp('Output vectors for input initialized filters:');
disp(['x_fo_ii_o = ', num2str(x_fo_ii_o(4:end))]);
disp(['y_fo_ii_o = ', num2str(y_fo_ii_o(4:end))]);
disp(['w_fo_ii_o = ', num2str(w_fo_ii_o(4:end))]);
disp('')
disp(['x_so_ii_o = ', num2str(x_so_ii_o(4:end))]);
disp(['y_so_ii_o = ', num2str(y_so_ii_o(4:end))]);
disp(['w_so_ii_o = ', num2str(w_so_ii_o(4:end))]);
disp('')
disp(['x_to_ii_o = ', num2str(x_to_ii_o(4:end))]);
disp(['y_to_ii_o = ', num2str(y_to_ii_o(4:end))]);
disp(['w_to_ii_o = ', num2str(w_to_ii_o(4:end))]);
disp('')
disp('Output vectors for input/output initialized filters:');
disp(['x_fo_io_o = ', num2str(x_fo_io_o(4:end))]);
disp(['y_fo_io_o = ', num2str(y_fo_io_o(4:end))]);
disp(['w_fo_io_o = ', num2str(w_fo_io_o(4:end))]);
disp('')
disp(['x_so_io_o = ', num2str(x_so_io_o(4:end))]);
disp(['y_so_io_o = ', num2str(y_so_io_o(4:end))]);
disp(['w_so_io_o = ', num2str(w_so_io_o(4:end))]);
disp('')
disp(['x_to_io_o = ', num2str(x_to_io_o(4:end))]);
disp(['y_to_io_o = ', num2str(y_to_io_o(4:end))]);
disp(['w_to_io_o = ', num2str(w_to_io_o(4:end))]);