experiment_name = "medium_hard"

log_dir = "exp_log"

agent_boss_a5_m5 = Agent("agent_cluster 10 .95 .0001 5 1 1 0 100 50 5 1 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_echo = Agent("echo run an agent")

agent_rmax5 = Agent("agent_vi_rmax 5 .95 .0001")
agent_rmax2 = Agent("agent_vi_rmax 2 .95 .0001")
agent_rmax10 = Agent("agent_vi_rmax 10 .95 .0001")

env_marble_simple_ap = Environment("glue_maze_env data/simple.xml cfgs/slip1.xml", cwd="/home/jasmuth/crl/mazes")
env_marble_small_ap = Environment("glue_maze_env data/small.xml cfgs/slip1.xml", cwd="/home/jasmuth/crl/mazes")

env_marble_small = Environment("glue_maze_env data/small.xml cfgs/slip001.xml", cwd="/home/jasmuth/crl/mazes")
env_medium_small = Environment("glue_maze_env data/medium.xml cfgs/slip001.xml", cwd="/home/jasmuth/crl/mazes")
env_marble_aaai = Environment("glue_maze_env data/aaai.xml cfgs/slip001.xml", cwd="/home/jasmuth/crl/mazes")
env_medium_hard = Environment("glue_maze_env data/medium_harder.xml cfgs/slip001.xml", cwd="/home/jasmuth/crl/mazes")

env_debug = Environment("echo run an environment")

num_steps = 2000
num_trials = 500
num_runs = 1

#instance names must be unique
instances = [
    Instance("BOSS_m5k5", agent_boss_a5_m5, env_medium_hard, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("RMAX_m5", agent_rmax5, env_medium_hard, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("RMAX_m2", agent_rmax2, env_medium_hard, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("RMAX_m10", agent_rmax10, env_medium_hard, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("BOSS_debug", agent_echo, env_marble_small, steps=num_steps, trials=num_trials, runs=num_runs),
    
]
