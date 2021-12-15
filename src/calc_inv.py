import numpy as np
from scipy import stats

calib_data_file = np.load("calibration_soln4.npz", allow_pickle=True)

r_x = calib_data_file['r_x']
r_z = calib_data_file['r_z']
t_x = calib_data_file['t_x']
t_y = calib_data_file['t_y']

A = np.vstack([t_x, t_x*t_x, t_y, t_y*t_y, np.ones_like(t_x)]).T
y = np.vstack([r_x, r_z]).T

soln = np.linalg.lstsq(A, y, rcond=None)

inv_x = soln[0][:,0]
inv_z = soln[0][:,1]

res_x = soln[1][0]
res_z = soln[1][1]

r2_x = 1 - res_x / (y.shape[0] * y[:,0].var())
r2_z = 1 - res_z / (y.shape[0] * y[:,1].var())

print("r2_x", r2_x)
print("r2_z", r2_z)

residuals_x = A @ inv_x - y[:,0]
residuals_z = A@ inv_z - y[:,1]

mean_x = np.mean(np.abs(residuals_x))
mean_z = np.mean(np.abs(residuals_z))

std_x = np.std(np.abs(residuals_x))
std_z = np.std(np.abs(residuals_z))

print("mean x ", mean_x)
print("std x ", std_x)
print("mean z ", mean_z)
print("std z ", std_z)

inv_soln = np.array(soln, dtype=object)

np.savez("inv_soln4.npz",r_x=r_x, r_z=r_z, t_x=t_x, t_y=t_y, inv_soln=inv_soln, inv_x = inv_x, inv_z=inv_z, res_x=res_x, res_z=res_z, r2_x=r2_x, r2_z=r2_z, mean_x =mean_x, mean_z=mean_z, std_x=std_x, std_z=std_z)