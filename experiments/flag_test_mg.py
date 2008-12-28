experiment_name = "flag_test"

log_dir = "exp_log"

maze = "data/small_rooms.xml"
cfg = "cfgs/slip001.xml"
cwd = "/home/marek/libcrlworkspace/mazes"
agent_echo = Agent("echo RUN YOUR AGENT")
agent_rtdp1 = Agent("expert_maze_agent "+maze+" "+cfg+" rtdp 1 0.1 25 500 10 0", cwd=cwd)
agent_vi = Agent("expert_maze_agent "+maze+" "+cfg+" vi 1 .001", cwd=cwd)
agent_ps = Agent("expert_maze_agent "+maze+" "+cfg+" ps 1 .001", cwd=cwd)

# Params for the uct agent:
# <gamma> <reward_type> <clear_tree> <full_tree> <run_limit> <time_limit> <C>
agent_uct = Agent("expert_maze_agent "+maze+" "+cfg+" uct 1 1 false true 500 0 0", cwd=cwd)

env = Environment("glue_maze_env "+maze+" "+cfg, cwd=cwd)

#instance names must be unique
instances = [
    Instance("uct", agent_uct, env, trials=10, runs=1),
    Instance("VI", agent_vi, env, trials=10, runs=1),
    #Instance("PS", agent_ps, env, trials=1000, runs=3),
    Instance("RTDP_25", agent_rtdp1, env, trials=10, runs=1)
 ]
