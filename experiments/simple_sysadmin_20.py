experiment_name = "simple_sysadmin"

log_dir = "exp_log"

agent_echo = Agent("echo run an agent")
agent_random = Agent("random_agent");
agent_null = Agent("null_agent");
#agent_spudd = Agent("glue_spudd /home/philipp/Desktop/experiments/Coordination_Discovery/simple_sysadmin_star_10/SPUDD-OPTDual-simplesysadmin-star-8.ADD", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext/libcrl")
#agent_alp = Agent("glue_alp ring 20 -t simple");
agent_alp1 = Agent("glue_alp ring 20 -t simple -w /home/philipp/Desktop/experiments/Coordination_Discovery/simple_sysadmin_ring_20/data_sort/weights0.txt", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext")
agent_alp2 = Agent("glue_alp ring 20 -t simple -w /home/philipp/Desktop/experiments/Coordination_Discovery/simple_sysadmin_ring_20/data_sort/weights23.txt -b /home/philipp/Desktop/experiments/Coordination_Discovery/simple_sysadmin_ring_20/data_sort/conj23.txt", cwd="/home/philipp/Builds/maastricht/3rdParty/libcrlext")

env_echo = Environment("echo run an environment")
glue_sysadmin1 = Environment("glue_sysadmin ring 20 -t simple")



num_steps = 30
num_trials = 50
num_runs = 10

#instance names must be unique
instances = [
    #Instance("DEBUG", agent_echo, env_echo, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("RAND", agent_random, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("NULL", agent_null, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("SPUDD", agent_spudd, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    #Instance("ALP", agent_alp, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("ALP1", agent_alp1, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
    Instance("ALP2", agent_alp2, glue_sysadmin1, steps=num_steps, trials=num_trials, runs=num_runs),
]
