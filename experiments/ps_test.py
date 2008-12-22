experiment_name = "ps_test"

log_dir = "exp_log"

agent_vi = Agent("expert_maze_agent data/big.xml cfgs/slip01.xml vi 1 .001",
                  cwd="/home/jasmuth/crl/mazes")
agent_ps = Agent("expert_maze_agent data/big.xml cfgs/slip01.xml ps 1 .001",
                  cwd="/home/jasmuth/crl/mazes")

env = Environment("glue_maze_env data/big.xml cfgs/slip01.xml",
                  cwd="/home/jasmuth/crl/mazes")

#instance names must be unique
instances = [
    #Instance("UCT", agent_uct, env, trials=1, runs=1),
    Instance("VI", agent_vi, env, trials=500, runs=1),
    Instance("PS", agent_ps, env, trials=500, runs=1)
]
