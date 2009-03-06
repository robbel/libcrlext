experiment_name = "littman_k"

log_dir = "exp_log"


agent_boss_k1 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 1 2 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_k2 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 2 2 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_k3 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 3 2 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_k4 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 4 2 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_k5 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 5 2 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_a5_m10_k6 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 6 2 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_a5_m10_k7 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 7 2 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_a5_m10_k8 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 8 2 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_a5_m10_k9 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 9 2 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_a5_m10_k10 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 10 2 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_a5_m10_k11 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 11 2 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_a5_m10_k12 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 12 2 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_a5_m10_k13 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 13 2 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_a5_m10_k14 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 14 2 0", cwd="/home/jasmuth/crl/cluster-boss")
agent_boss_a5_m10_k15 = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 15 2 0", cwd="/home/jasmuth/crl/cluster-boss")

env_littman1 = Environment("env_littman cfgs/littman1.xml", cwd="/home/jasmuth/crl/glue-environments")
env_littman2 = Environment("env_littman cfgs/littman2.xml", cwd="/home/jasmuth/crl/glue-environments")
env_debug = Environment("echo run an environment")


num_steps = 200
num_trials = 50
num_runs = 1

instances = []
for k in range(1, 16):
    agent_boss_k = Agent("agent_cluster 5 1 .1 10 1 1 0 100 50 "+`k`+" 2 0", cwd="/home/jasmuth/crl/cluster-boss")
    instances.append(Instance("BOSS_k"+`k`, agent_boss_k, env_littman2, steps=num_steps, trials=num_trials, runs=num_runs)),
