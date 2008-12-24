experiment_name = "uct_test"

log_dir = "exp_log"

maze = "data/aaai.xml"
cfg = "cfgs/slip01.xml"
cwd = "/home/jasmuth/crl/mazes"

agent_uct = Agent("expert_maze_agent "+maze+" "+cfg+" uct .9 1 0 500",
                  cwd=cwd)
agent_vi = Agent("expert_maze_agent "+maze+" "+cfg+" vi 1 .001",
                  cwd=cwd)

env = Environment("glue_maze_env "+maze+" "+cfg,
                  cwd=cwd)

#instance names must be unique
instances = [
    Instance("UCT", agent_uct, env, trials=1, runs=1),
    Instance("VI", agent_vi, env, trials=50, runs=1)
]
