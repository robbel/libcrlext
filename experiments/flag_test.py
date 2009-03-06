experiment_name = "flag_test"

log_dir = "exp_log"

maze = "data/big.xml"
cfg = "cfgs/slip001.xml"
cwd = "/home/jasmuth/crl/mazes"
agent_echo = Agent("echo RUN YOUR AGENT")
agent_rtdp1 = Agent("expert_maze_agent "+maze+" "+cfg+" rtdp 1 0.1 25 500 100 1", cwd=cwd)
agent_rtdp2 = Agent("expert_maze_agent "+maze+" "+cfg+" rtdp 1 0.1 500 500 100 1", cwd=cwd)
agent_vi = Agent("expert_maze_agent "+maze+" "+cfg+" vi 1 .001", cwd=cwd)
agent_ps = Agent("expert_maze_agent "+maze+" "+cfg+" ps 1 .001", cwd=cwd)

env = Environment("glue_maze_env "+maze+" "+cfg, cwd=cwd)

#instance names must be unique
instances = [
    Instance("VI", agent_vi, env, trials=10, runs=1),
    Instance("PS", agent_ps, env, trials=10, runs=1),
    #Instance("echo", agent_echo, env, trials=100, runs=1),
    #Instance("PS", agent_ps, env, trials=1000, runs=3),
    #Instance("RTDP_25", agent_rtdp1, env, trials=100, runs=1),
    #Instance("RTDP_500", agent_rtdp2, env, trials=100, runs=6),
]
