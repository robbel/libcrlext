experiment_name = "littman"

log_dir = "exp_log"

agent_boss_a5_m20 = Agent("agent_cluster 5 1 .1 20 1 1 0 100 50 5 2", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_a5_m10 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 5 2", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_a5_m5 = Agent("agent_cluster 5 1 .1 5 1 1 0 100 50 5 2", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_a5_m2 = Agent("agent_cluster 5 1 .1 2 1 1 0 100 50 5 2", cwd="/home/jasmuth/crl/cluster-boss")
agent_echo = Agent("echo run an agent")

agent_rmax2 = Agent("agent_vi_rmax 2 1 .1")
agent_rmax5 = Agent("agent_vi_rmax 5 1 .1")
agent_rmax10 = Agent("agent_vi_rmax 10 1 .1")
agent_rmax20 = Agent("agent_vi_rmax 20 1 .1")

agent_random = Agent("random_agent")

env_littman = Environment("env_littman cfgs/littman2.xml", cwd="/home/jasmuth/crl/glue-environments")
env_debug = Environment("echo run an environment")


num_steps = 200
num_trials = 50
num_runs = 1

#instance names must be unique
instances = [

    Instance("BOSS_a5_m20", agent_boss_a5_m20, env_littman, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("BOSS_a5_m10", agent_boss_a5_m10, env_littman, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("BOSS_a5_m5", agent_boss_a5_m5, env_littman, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("BOSS_a5_m2", agent_boss_a5_m2, env_littman, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("RAND", agent_random, env_debug, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("RMAX_m2", agent_rmax2, env_littman, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("RMAX_m5", agent_rmax5, env_littman, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("RMAX_m10", agent_rmax10, env_littman, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("RMAX_m20", agent_rmax20, env_littman, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("DEBUG", agent_echo, env_littman, steps=num_steps, trials=num_trials, runs=num_runs),
]
