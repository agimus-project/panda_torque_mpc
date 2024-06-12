import os
import numpy as np
import mpc_utils
import pin_utils
import example_robot_data
import pinocchio as pin
import initialize_croco_reaching as croco_reach
from panda_torque_mpc_pywrap import CrocoddylReaching
from read_plot_utils import read_jsid_bag

DIRECTORY = "."  # ../bags/

CONTROLLER_NAME = "ctrl_mpc_linearized"
BAG_NAMES = [
    f"{CONTROLLER_NAME}_expe.bag",
]


BAG_PATHS = [os.path.join(DIRECTORY, name) for name in BAG_NAMES]


croco_reaching, params, robot_model = croco_reach.get_croco_reaching()

robot = example_robot_data.load("panda")
robot.model = robot_model

data = read_jsid_bag(BAG_PATHS[0], CONTROLLER_NAME)
data["t"] += data["target_time"][0]

first_solve = True
xs_0 = np.concatenate([data["q"]["measured"][0], data["dq"]["measured"][0]])
xs_init = (params["nb_shooting_nodes"] + 1) * [xs_0]
tau_grav = pin.computeGeneralizedGravity(robot_model, robot.data, np.array(xs_0[:7]))
tau_grav = tau_grav[:7]
us_init = params["nb_shooting_nodes"] * [tau_grav]
# # # # # # # # # # # #
###  MPC SIMULATION ###
# # # # # # # # # # # #
# OCP parameters
ocp_params = {}
ocp_params["N_h"] = params["nb_shooting_nodes"]
ocp_params["dt"] = params["dt_ocp"]
ocp_params["maxiter"] = 1
ocp_params["pin_model"] = robot_model
ocp_params["armature"] = params["armature"]
ocp_params["id_endeff"] = robot_model.getFrameId("panda_joint7")
ocp_params["active_costs"] = croco_reaching.solver.problem.runningModels[
    0
].differential.costs.active.tolist()

# Simu parameters
sim_params = {}
dt_sim = (data["t"][-1] - data["t"][0]) / len(data["t"])
sim_params["sim_freq"] = int(1 / dt_sim)
sim_params["mpc_freq"] = int(1 / dt_sim)

sim_params["T_sim"] = 6
log_rate = 100
# Initialize simulation data
robot.x0 = xs_0
sim_data = mpc_utils.init_sim_data(sim_params, ocp_params, robot.x0)

mpc_cycle = 0
for i in range(sim_data["N_sim"]):

    if i % log_rate == 0:
        print("\n SIMU step " + str(i) + "/" + str(sim_data["N_sim"]) + "\n")

    # Solve OCP if we are in a planning cycle (MPC/planning frequency)
    if i % int(sim_params["sim_freq"] / sim_params["mpc_freq"]) == 0:
        # Set x0 to measured state

        x0 = np.concatenate([data["q"]["measured"][i], data["dq"]["measured"][i]])
        croco_reaching.solver.problem.x0 = x0
        time = data["t"][i]

        croco_reaching.set_ee_ref_placement(time, True, 1.0)
        croco_reaching.set_posture_ref(x0)

        # Warm start using previous solution
        if first_solve:
            xs_init = [np.array(x) for x in xs_init]
            us_init = [np.array(u) for u in us_init]
            croco_reaching.solve(xs_init, us_init, 500)
        else:
            xs_init = [np.array(x) for x in croco_reaching.solver.xs]
            us_init = [np.array(x) for x in croco_reaching.solver.us]
            croco_reaching.solve(xs_init, us_init, 1)

        # Solve OCP & record MPC predictions

        xs = croco_reaching.solver.xs
        us = croco_reaching.solver.us
        sim_data["state_pred"][mpc_cycle, :, :] = np.array(xs)
        sim_data["ctrl_pred"][mpc_cycle, :, :] = np.array(us)
        # Extract relevant predictions for interpolations
        x_curr = sim_data["state_pred"][
            mpc_cycle, 0, :
        ]  # x0* = measured state    (q^,  v^ )
        x_pred = sim_data["state_pred"][
            mpc_cycle, 1, :
        ]  # x1* = predicted state   (q1*, v1*)
        u_curr = sim_data["ctrl_pred"][
            mpc_cycle, 0, :
        ]  # u0* = optimal control   (tau0*)
        # Record costs references
        q = sim_data["state_pred"][mpc_cycle, 0, : sim_data["nq"]]
        sim_data["ctrl_ref"][mpc_cycle, :] = pin_utils.get_u_grav(
            q,
            croco_reaching.solver.problem.runningModels[0].differential.pinocchio,
            ocp_params["armature"],
        )
        sim_data["state_ref"][mpc_cycle, :] = (
            croco_reaching.solver.problem.runningModels[0]
            .differential.costs.costs["state_reg"]
            .cost.residual.reference
        )
        sim_data["lin_pos_ee_ref"][mpc_cycle, :] = (
            croco_reaching.solver.problem.runningModels[0]
            .differential.costs.costs["placement_cost"]
            .cost.residual.reference.translation
        )
        # m0=croco_reaching.solver.problem.runningModels[0].differential.costs
        # Select reference control and state for the current MPC cycle
        x_ref_MPC_RATE = x_curr + sim_data["ocp_to_mpc_ratio"] * (x_pred - x_curr)
        u_ref_MPC_RATE = u_curr
        if mpc_cycle == 0:
            sim_data["state_des_MPC_RATE"][mpc_cycle, :] = x_curr
        sim_data["ctrl_des_MPC_RATE"][mpc_cycle, :] = data["tau"]["commanded"][
            i
        ]  # u_ref_MPC_RATE
        sim_data["state_des_MPC_RATE"][mpc_cycle + 1, :] = np.concatenate(
            [data["q"]["commanded"][i], data["dq"]["commanded"][i]]
        )  # x_ref_MPC_RATE

        # Increment planning counter
        mpc_cycle += 1

        # Select reference control and state for the current SIMU cycle
        # x_ref_SIM_RATE = x_curr + sim_data["ocp_to_mpc_ratio"] * (x_pred - x_curr)
        # u_ref_SIM_RATE = u_curr

        # First prediction = measurement = initialization of MPC
        if i == 0:
            sim_data["state_des_SIM_RATE"][i, :] = x_curr
        sim_data["ctrl_des_SIM_RATE"][i, :] = data["tau"]["commanded"][i]
        sim_data["state_des_SIM_RATE"][i + 1, :] = np.concatenate(
            [data["q"]["commanded"][i], data["dq"]["commanded"][i]]
        )

        sim_data["state_mea_SIM_RATE"][i + 1, :] = np.concatenate(
            [data["q"]["measured"][i + 1], data["dq"]["measured"][i + 1]]
        )  # x_mea_SIM_RATE


plot_data = mpc_utils.extract_plot_data_from_sim_data(sim_data)

mpc_utils.plot_mpc_results(
    plot_data,
    which_plots=["all"],
    PLOT_PREDICTIONS=True,
    pred_plot_sampling=int(sim_params["mpc_freq"] / 10),
)
