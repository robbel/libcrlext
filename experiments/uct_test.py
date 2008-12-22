experiment_name = "uct_test"

log_dir = "exp_log"

agent_uct = Agent("expert_maze_agent data/aaai.xml cfgs/slip01.xml uct .9 1 500 0",
                  cwd="/home/jasmuth/crl/mazes")
agent_vi = Agent("expert_maze_agent data/aaai.xml cfgs/slip01.xml vi 1 .001",
                  cwd="/home/jasmuth/crl/mazes")

env = Environment("glue_maze_env data/aaai.xml cfgs/slip01.xml",
                  cwd="/home/jasmuth/crl/mazes")

#instance names must be unique
instances = [
    #Instance("UCT", agent_uct, env, trials=1, runs=1),
    Instance("VI", agent_vi, env, trials=50, runs=1)
]
