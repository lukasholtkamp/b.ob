import numpy as np
import matplotlib.pyplot as plt
import math

from NMPC_solver import *

# Define constants and parameters
curvatures = np.linspace(0, 2, 11)      # Curvature values η_k from 0 to 3
d_k = 0.2                                  # Controls the length of the path segment
theta_steps = 10                         # Number of discrete points along θ for each path
# grid_resolution = 2                     # Number of samples along each axis (tangential, normal, orientation)
grid_resolution = 11                     # Number of samples along each axis (tangential, normal, orientation)

width= 0.4
length = 0.5

v_max = 0.1

def gamma(eta):
    if eta==0:
        return 1
    else:
        return 0.5*(np.sqrt(1+(2*eta)**2)+((np.arcsinh(2*eta))/(2*eta)))

def g(v_max,eta):
    return v_max/gamma(eta)

# Function to generate a parabolic path given η
def generate_path(eta):
    g_eta = g(v_max,eta)
    theta_vals = np.linspace(-d_k/g_eta, d_k/g_eta, theta_steps)  # Adjusted theta range
    path_points = [(g_eta * theta, eta * (g_eta * theta)**2) for theta in theta_vals]
    return theta_vals, np.array(path_points)

# Placeholder for solving the optimal control problem for a given state and path
def compute_optimal_control(state, path_segment):
    # Use an OCP solver like CasADi to calculate the optimal control (s, ω, v)
    # For now, we'll just return some dummy values for illustration
    s = np.random.uniform(0, 1)
    omega = np.random.uniform(-0.5, 0.5)
    v = np.random.uniform(0, 1)
    return s, omega, v
        
# Lists to store training data
states = []
controls = []

m_normal = lambda s: -1/(2*curvature*s*g(v_max,curvature))
y_normal = lambda s,x: m_normal(s)*x - m_normal(s)*s*g(v_max,curvature) + curvature*(s*g(v_max,curvature))**2

m_tangent = lambda s: (2*curvature*s*g(v_max,curvature))
y_tangent = lambda s,x: m_tangent(s)*x - m_tangent(s)*s*g(v_max,curvature) + curvature*(s*g(v_max,curvature))**2

cir_r = 0.03
bot_r = 0
outside_region = 0.05 # test 
cases = [None,0.1,0,-0.1]

def in_circle(xc,yc,rc,x,y,r):
    distance = math.sqrt((xc - x) ** 2 + (yc - y) ** 2)
    
    # Check if the distance is less than or equal to the sum of their radii
    if distance <= (rc + r) or distance > (rc+outside_region):
        return True  # Circles collide
    else:
        return False  # Circles do not collide
    

for curvature in curvatures:

    th,zeta = generate_path(curvature)

    for k in range(1): #len(th)

        tangent_axis = np.linspace(th[k]-length/2,th[k]+length/2,grid_resolution)

        if grid_resolution%2==0:
            normal_axis = np.linspace((width)/(2*(grid_resolution-1)),width/2,int(grid_resolution/2))
        else:
            normal_axis = np.linspace(0,width/2,int(grid_resolution/2)+1)

        for case in cases:

            fig, ax = plt.subplots()

            states = []
            controls = []

            if case!= None:

                if curvature>0:
                    dx = case*np.cos(np.arctan(m_normal(th[k])))
                    xc = zeta[k,0] - dx
                    yc = y_normal(th[k],xc)
                else:
                    dx = case
                    xc = zeta[k,0]
                    yc = -dx

                circle = plt.Circle((xc,yc), cir_r, color='blue', fill=False, linewidth=2)  # 'fill=False' makes it a hollow circle

                # Add the circle to the plot
                ax.add_patch(circle)

                ax.plot(xc,yc,'g*')
            

            for i in range(len(tangent_axis)):

                for j in range(len(normal_axis)):

                    start_angle = np.arctan(2 * curvature * (tangent_axis[i])*g(v_max,curvature))
                    
                    # Generate 10 angles around the circle, starting from the start_angle
                    angles = np.linspace(start_angle, start_angle + 2 * np.pi,grid_resolution, endpoint=False)

                    # Apply modulus to ensure all angles are within [0, 2*pi)
                    angles = angles % (2 * np.pi)

                    for orientation in angles:

                        if curvature>0:
                            dx = np.abs(normal_axis[j]*np.cos(np.arctan(m_normal(tangent_axis[i]))))

                            x_l = (tangent_axis[i])*g(v_max,curvature) - dx
                            x_r = (tangent_axis[i])*g(v_max,curvature) + dx

                            if normal_axis[j]!=0:
                                if case== None:
                                    states.append([x_l,y_normal(tangent_axis[i],x_l),orientation,th[k],curvature])
                                    states.append([x_r,y_normal(tangent_axis[i],x_r),orientation,th[k],curvature])
                                    
                                elif case==0:
                                    
                                    if not in_circle(xc,yc,cir_r,x_l,y_normal(tangent_axis[i],x_l),bot_r):
                                        states.append([x_l,y_normal(tangent_axis[i],x_l),orientation,th[k],curvature])

                                    if not in_circle(xc,yc,cir_r,x_r,y_normal(tangent_axis[i],x_r),bot_r):
                                        states.append([x_r,y_normal(tangent_axis[i],x_r),orientation,th[k],curvature])

                                elif case>0:
                                    
                                    if not in_circle(xc,yc,cir_r,x_l,y_normal(tangent_axis[i],x_l),bot_r):
                                        states.append([x_l,y_normal(tangent_axis[i],x_l),orientation,th[k],curvature])
                                else:
                                    if not in_circle(xc,yc,cir_r,x_r,y_normal(tangent_axis[i],x_r),bot_r):
                                        states.append([x_r,y_normal(tangent_axis[i],x_r),orientation,th[k],curvature])
                                    
                                # x_pred,u = run_open_loop_mpc(v_max, [x_l,y_normal(tangent_axis[i],x_l),orientation], th[k],curvature)
                                # controls.append(u[0,:])
                                # plt.plot(x_pred[:, 0], x_pred[:, 1], color='green')

                                # x_pred,u = run_open_loop_mpc(v_max, [x_r,y_normal(tangent_axis[i],x_r),orientation], th[k],curvature)
                                # controls.append(u[0,:])
                                # plt.plot(x_pred[:, 0], x_pred[:, 1], color='green')

                            else:
                                if case== None:
                                    states.append([x_l,y_normal(tangent_axis[i],x_l),orientation,th[k],curvature])
                                else:
                                    if not in_circle(xc,yc,cir_r,x_l,y_normal(tangent_axis[i],x_l),bot_r):
                                        states.append([x_l,y_normal(tangent_axis[i],x_l),orientation,th[k],curvature])

                                # x_pred,u = run_open_loop_mpc(v_max, [x_l,y_normal(tangent_axis[i],x_l),orientation], th[k],curvature)
                                # controls.append(u[0,:])
                                # plt.plot(x_pred[:, 0], x_pred[:, 1], color='green')


                        else:
                            dx = normal_axis[j]

                            x_l = (tangent_axis[i])*g(v_max,curvature)
                            x_r = (tangent_axis[i])*g(v_max,curvature)

                            if normal_axis[j]!=0:
                                if case== None:
                                    states.append([x_l,-dx,orientation,th[k],curvature])
                                    states.append([x_r,dx,orientation,th[k],curvature])
                                
                                elif case==0:

                                    if not in_circle(xc,yc,cir_r,x_l,-dx,bot_r):
                                        states.append([x_l,-dx,orientation,th[k],curvature])

                                    if not in_circle(xc,yc,cir_r,x_r,dx,bot_r):
                                        states.append([x_r,dx,orientation,th[k],curvature])

                                elif case>0:
                                    if not in_circle(xc,yc,cir_r,x_l,-dx,bot_r):
                                        states.append([x_l,-dx,orientation,th[k],curvature])
                                else:
                                    if not in_circle(xc,yc,cir_r,x_r,dx,bot_r):
                                        states.append([x_r,dx,orientation,th[k],curvature])

                                # x_pred,u = run_open_loop_mpc(v_max, [x_l,-dx,orientation], th[k],curvature)
                                # controls.append(u[0,:])
                                # plt.plot(x_pred[:, 0], x_pred[:, 1], color='green')

                                # x_pred,u = run_open_loop_mpc(v_max, [x_r,dx,orientation], th[k],curvature)
                                # controls.append(u[0,:])
                                # plt.plot(x_pred[:, 0], x_pred[:, 1], color='green')
                            else:
                                if case== None:
                                    states.append([x_l,0,orientation,th[k],curvature])
                                else:
                                    if not in_circle(xc,yc,cir_r,x_l,0,bot_r):
                                        states.append([x_l,0,orientation,th[k],curvature])

                                # x_pred,u = run_open_loop_mpc(v_max, [x_l,0,orientation], th[k],curvature)
                                # controls.append(u[0,:])
                                # plt.plot(x_pred[:, 0], x_pred[:, 1], color='green')
            
            # states = np.array(states)
            # controls = np.array(controls)

            # Loop through each point
            for x, y, orientation,_,_ in states:
                # Plot the point
                ax.plot(x, y, 'bo')  # 'bo' for blue dots

                # Plot the smaller arrow for orientation
                scale = 0.0001  # Scale factor for arrow size
                dx = scale * np.cos(orientation)  # x-component of the arrow
                dy = scale * np.sin(orientation)  # y-component of the arrow
                ax.arrow(x, y, dx, dy, head_width=0.0001, head_length=0.005, fc='r', ec='r')  # Smaller red arrow

            # plt.plot(states[:,0],states[:,1],'r*')
            # ax.plot(zeta[k,0],zeta[k,1],'ko')
            ax.plot(zeta[:,0],zeta[:,1])

            ax = plt.gca()
            ax.set_aspect('equal', adjustable='box')

            plt.show()

# Save the data or use it directly for training
np.save("train_states.npy", states)
np.save("train_controls.npy", controls)
