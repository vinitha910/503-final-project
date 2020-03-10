import numpy as np
from cma import CMA

OPT_DIM = 8

def random_params():
    return np.random.uniform(low=0.0, high=1.0, size=OPT_DIM) * [20, 20, 100, 100, 20, 20, 100, 100]

def cma_validate(planner_fn):
    found_error = False
    failing_test = None
    def f(env_parameters, render=None):
        error, success, num_expansions, planning_time = planner_fn(env_parameters, render)
        if error:
            nonlocal found_error
            nonlocal failing_test
            found_error = True
            failing_test = env_parameters
        return -num_expansions

    max_chains = 1 # TODO random restarts

    for i in range(max_chains):
        initial_mean = np.array([20, 5, 57, 58, 5, 5, 44, 85, 40, 70, 70, 80])
        #initial_mean = random_params() # TODO random restarts
        initial_sigma = 2.0
        initial_cov = np.eye(len(initial_mean))
        opzer = CMA(f, initial_mean, initial_sigma, initial_cov)
        max_iters = 50
        min_val = 0
        convergence_patience = 5
        remaining_patience = convergence_patience
        i = 0
        while True:
            print("Iteration", i, "Patience", remaining_patience)
            val = f(opzer.mean[:,0], render=True)
            if found_error:
                return False, failing_test
            if (val < min_val):
                remaining_patience = convergence_patience
                min_val = val
            else:
                remaining_patience = remaining_patience - 1
                if remaining_patience == 0:
                    break
            if i > max_iters:
                break
            i += 1
            opzer.iter()

    return True, None

def random_validate(planner_fn):
    max_iters = 300
    for i in range(max_iters):
        env_parameters = random_params()
        error, success, num_expansions, planning_time = planner_fn(env_parameters, True)
        if error:
            return False, env_parameters
    return True, None

PARAM_FN = "env_config.txt"
RENDER_FN = "render.png"

def human_validate(planner_fn):
    instructions = ("\nWelcome, human!\n"
                    "Please tell us whether this motion planning algorithm is valid (i.e. has no bugs).\n"
                    "Type one of the following commands:\n\n"
                    "  random -- writes a random environment config to 'env_config.txt'\n"
                    "            (feel free to manually edit this file to tweak the environment config)\n"
                    "  test   -- reads the environment config from 'env_config.txt', runs the planner\n"
                    "            on that environment, and reports the results\n"
                    "  exit   -- declare that the planning algorithm is valid (terminate this program)\n\n"
                    "If during an execution of `test` the planner crashes, times out, or returns a non-\n"
                    "optimal plan, then you have successfully found a bug, and this program will\n"
                    "automatically terminate.")
    format_explanation = "Format: two rectangular obstacles specified as [l1, w1, x1, y1, l2, w2, x2, y2]\nThe point (0,0) is in the upper-left corner and (100,100) in the lower-right."
    print(instructions)
    while True:
        cmd = input("\nEnter a command (random, test, exit): ")
        if cmd == "random":
            params = random_params().astype(int)
            np.savetxt(PARAM_FN, params, fmt='%d', header=format_explanation)
            print("Saved to " + PARAM_FN)
        elif cmd == "test":
            params = np.loadtxt(PARAM_FN)
            print("Executing config:")
            print("\n".join(params.astype(int).astype(str)))
            error, success, num_expansions, planning_time = planner_fn(params, RENDER_FN)
            print("You can open " + RENDER_FN + " to visualize the result.")
            # The result is already printed by planner_fn. No need to print it again.
            if error:
                return False, params
        elif cmd == "exit":
            return True, None
        else:
            print("Unrecognized command")


