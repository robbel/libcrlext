experiment_name = "graphprop"

log_dir = "exp_log"

agent_echo = Agent("echo run an agent")
agent_random = Agent("random_agent");

env_echo = Environment("echo run an environment")
glue_graphprop1 = Environment("glue_graphprop cfgs/default.xml data/Gnp100.txt", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext/graphprop")


num_steps = 200
num_trials = 50
num_runs = 5

#instance names must be unique
instances = [
    #Instance("DEBUG", agent_echo, env_echo, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("RAND", agent_random, glue_graphprop1, steps=num_steps, trials=num_trials, runs=num_runs),
]
