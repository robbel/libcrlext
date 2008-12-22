experiment_name = "rmax_vi_test"

log_dir = "exp_log"

agent = Agent("agent_vi_rmax 5 1 .001")

env = Environment("glue_maze_env data/big.xml cfgs/slip001.xml",
                  cwd="/home/jasmuth/crl/mazes")

#instance names must be unique
instances = [
    Instance("RMAX_VI", agent, env, trials=350, runs=1)
]
