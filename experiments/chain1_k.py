experiment_name = "chain1_k"

log_dir = "exp_log"
agent_echo = Agent("echo run an agent")

agent_rmax2 = Agent("agent_vi_rmax 2 .95 .1")
agent_rmax5 = Agent("agent_vi_rmax 5 .95 .1")
agent_rmax10 = Agent("agent_vi_rmax 10 .95 .1")

env_chain = Environment("env_poupart cfgs/chain1.xml", cwd="/Users/jasmuth/Documents/libcrl/glue-environments")
env_debug = Environment("echo run an environment")

num_steps = 1000
num_trials = 1
num_runs = 2

#instance names must be unique
instances = []
for k in [5, 10]:
    agent_boss_k = Agent("agent_cluster 1 .95 .1 10 1 1 0 100 50 "+`k`+" 0 0", cwd="/Users/jasmuth/Documents/libcrl/cluster-boss")
    instances.append(Instance("BOSS_k"+`k`, agent_boss_k, env_chain, steps=num_steps, trials=num_trials, runs=num_runs)),
instances.append(Instance("RMAX_m5", agent_rmax5, env_chain, steps=num_steps, trials=num_trials, runs=num_runs)),
