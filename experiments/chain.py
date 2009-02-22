experiment_name = "chain"

log_dir = "exp_log"

agent_random = Agent("agent_cluster 5 .9 .1")

env = Environment("env_poupart")

#instance names must be unique
instances = [
    Instance("RAND", agent_random, env, trials=100, runs=1)
]
