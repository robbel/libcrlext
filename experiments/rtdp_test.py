experiment_name = "rtdp_test"

log_dir = "exp_log"

maze = "data/big.xml"
cfg = "cfgs/slip001.xml"
cwd = "/home/jasmuth/crl/mazes"
agent_echo = Agent("echo RUN YOUR AGENT")
agent_rtdp1 = Agent("expert_maze_agent "+maze+" "+cfg+" rtdp 1 0.001 25 500 10 0", cwd=cwd)
agent_rtdp1h = Agent("expert_maze_agent "+maze+" "+cfg+" rtdp 1 0.001 25 500 10 1", cwd=cwd)
agent_rtdp2 = Agent("expert_maze_agent "+maze+" "+cfg+" rtdp 1 0.001 50 500 10 0", cwd=cwd)
agent_rtdp3 = Agent("expert_maze_agent "+maze+" "+cfg+" rtdp 1 0.001 100 500 10 0", cwd=cwd)
agent_rtdp4 = Agent("expert_maze_agent "+maze+" "+cfg+" rtdp 1 0.001 500 500 10 0", cwd=cwd)
agent_vi = Agent("expert_maze_agent "+maze+" "+cfg+" vi 1 .001", cwd=cwd)
agent_ps = Agent("expert_maze_agent "+maze+" "+cfg+" ps 1 .001", cwd=cwd)

env = Environment("glue_maze_env "+maze+" "+cfg, cwd=cwd)

#instance names must be unique
instances = [
    Instance("VI", agent_vi, env, trials=2000, runs=3),
    #Instance("PS", agent_ps, env, trials=1000, runs=3),
    Instance("RTDP_25", agent_rtdp1, env, trials=2000, runs=3),
    Instance("RTDP_25_H", agent_rtdp1h, env, trials=2000, runs=3),
    #Instance("RTDP_50", agent_rtdp2, env, trials=1000, runs=6),
    #Instance("RTDP_100", agent_rtdp3, env, trials=1000, runs=6),
    #Instance("RTDP_500", agent_rtdp4, env, trials=1000, runs=6)
    #Instance("RTDP-debug", agent_echo, env, trials=500, runs=1)
]
