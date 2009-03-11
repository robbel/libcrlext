experiment_name = "chain1_k"

log_dir = "exp_log"
#5 .95 .1 20 1 1 0 100 50 5 2 0
agent_boss1500m5s10 = Agent("agent_cluster .5 .95 .1 5 1 1 0 1500 150 10 0 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss1000m5s10 = Agent("agent_cluster .5 .95 .1 5 1 1 0 1000 100 10 0 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss500m5s10 = Agent("agent_cluster .5 .95 .1 5 1 1 0 500 50 10 0 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss500m10s10 = Agent("agent_cluster .5 .95 .1 10 1 1 0 500 50 10 0 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss500m5s5 = Agent("agent_cluster .5 .95 .1 5 1 1 0 500 50 5 0 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss500m10s5 = Agent("agent_cluster .5 .95 .1 10 1 1 0 500 50 5 0 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_echo = Agent("echo run an agent")

agent_rmax2 = Agent("agent_vi_rmax 2 .95 .1")
agent_rmax5 = Agent("agent_vi_rmax 5 .95 .1")
agent_rmax10 = Agent("agent_vi_rmax 10 .95 .1")

env_chain = Environment("env_poupart cfgs/chain1.xml", cwd="/home/jasmuth/crl/glue-environments")
env_debug = Environment("echo run an environment")

num_steps = 1000
num_trials = 1
num_runs = 2

#instance names must be unique
instances = []
for k in [1, 2, 5, 10, 15, 20, 40, 100]:
    agent_boss_k = Agent("agent_cluster 1 .95 .1 10 1 1 0 100 50 "+`k`+" 0 0", cwd="/home/jasmuth/crl/cluster-boss")
    instances.append(Instance("BOSS_k"+`k`, agent_boss_k, env_chain, steps=num_steps, trials=num_trials, runs=num_runs)),
instances.append(Instance("RMAX_m5", agent_rmax5, env_chain, steps=num_steps, trials=num_trials, runs=num_runs)),
