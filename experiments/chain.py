experiment_name = "chain"

log_dir = "exp_log"

agent_boss1500m5s10 = Agent("agent_cluster .5 .95 .1 5 1 1 0 1500 150 10 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss1000m5s10 = Agent("agent_cluster .5 .95 .1 5 1 1 0 1000 100 10 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss500m5s10 = Agent("agent_cluster .5 .95 .1 5 1 1 0 500 50 10 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss500m10s10 = Agent("agent_cluster .5 .95 .1 10 1 1 0 500 50 10 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss500m5s5 = Agent("agent_cluster .5 .95 .1 5 1 1 0 500 50 5 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss500m10s5 = Agent("agent_cluster .5 .95 .1 10 1 1 0 500 50 5 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_echo = Agent("echo run an agent")

agent_rmax2 = Agent("agent_vi_rmax 2 .95 .1")
agent_rmax5 = Agent("agent_vi_rmax 5 .95 .1")
agent_rmax10 = Agent("agent_vi_rmax 10 .95 .1")

env_chain = Environment("env_poupart cfgs/chain1.xml", cwd="/home/jasmuth/crl/glue-environments")
env_debug = Environment("echo run an environment")

num_steps = 20
num_trials = 10
num_runs = 1

#instance names must be unique
instances = [
    Instance("BOSS_test", agent_boss500m5s5, env_chain, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("BOSS_b1500_m5_s10", agent_boss1500m5s10, env_chain, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("BOSS_b1000_m5_s10", agent_boss1000m5s10, env_chain, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("BOSS_b500_m5_s5", agent_boss500m5s5, env_chain, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("BOSS_b500_m10_s5", agent_boss500m10s5, env_chain, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("BOSS_b500_m5_s10", agent_boss500m5s10, env_chain, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("BOSS_b500_m10_s10", agent_boss500m10s10, env_chain, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("RMAX_m2", agent_rmax2, env_chain, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("RMAX_m5", agent_rmax5, env_chain, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("RMAX_m10", agent_rmax10, env_chain, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("DEBUG", agent_echo, env_chain, steps=num_steps, trials=num_trials, runs=num_runs),
]
