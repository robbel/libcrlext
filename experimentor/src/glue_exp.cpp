/*
 Copyright (C) 2008, Brian Tanner
 modified by John Asmuth
 modified by Philipp Robbel

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

     http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */

#include <iostream>
#include <fstream>
#include <sstream>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <rlglue/RL_glue.h>
#include <rlglue/utils/C/RLStruct_util.h>
#include <rlglue/Environment_common.h>

using namespace std;

typedef double reward_t;
typedef enum {xml, csv} output_t;

output_t output;
bool experiment_verbose;

inline time_t time_in_milli() {
  struct timeval tv;
  gettimeofday(&tv, 0);
  return tv.tv_usec/1000+tv.tv_sec*1000;
}

template <class T>
string printArray(const char* name, T* a, int n) {
	ostringstream os;
	os << "<" << name << ">";
	for (int i=0; i<n; i++) {
		os << a[i];
		if (i != n-1)
			os << ", ";
	}
	os << "</" << name << ">";
	return os.str();
}

void runEpisode(ostream& os, int episode, int step_limit, int& steps_total, reward_t& cumulative, time_t& total_time, int& term_total) {
	int terminal;
	if (output == xml)
		os << "  <Episode>" << endl;

	time_t start_time = time_in_milli();
	if (!experiment_verbose) {
		terminal = RL_episode(step_limit);
	}
	else {
		os << "   <Trajectory>" << endl;
		const observation_action_t* startResponse = RL_start();
		start_time = time_in_milli();
		os << "    <Observation>" << endl;
		if (startResponse->observation->numInts)
			os << "     " << printArray<int>("ints", startResponse->observation->intArray, startResponse->observation->numInts) << endl;
		if (startResponse->observation->numDoubles)
			os << "     " << printArray<double>("doubles", startResponse->observation->doubleArray, startResponse->observation->numDoubles) << endl;
		if (startResponse->observation->numChars)
			os << "     " << printArray<char>("chars", startResponse->observation->charArray, startResponse->observation->numChars) << endl;
		os << "    </Observation>" << endl;


		int steps = 1;

		const reward_observation_action_terminal_t* stepResponse = RL_step();

		while(stepResponse->terminal != 1 && (step_limit == 0 || steps < step_limit)){
			stepResponse = RL_step();
			if(stepResponse->terminal !=1 ){
				os << "    <Action>" << endl;
				if (startResponse->action->numInts)
					os << "     " << printArray<int>("ints", stepResponse->action->intArray, stepResponse->action->numInts) << endl;
				if (startResponse->action->numDoubles)
					os << "     " << printArray<double>("doubles", stepResponse->action->doubleArray, stepResponse->action->numDoubles) << endl;
				if (startResponse->action->numChars)
					os << "     " << printArray<char>("chars", stepResponse->action->charArray, stepResponse->action->numChars) << endl;
				os << "    </Action>" << endl;
				os << "    <reward>" << stepResponse->reward << "</reward>" << endl;
				os << "    <Observation>" << endl;
				if (startResponse->observation->numInts)
					os << "     " << printArray<int>("ints", stepResponse->observation->intArray, stepResponse->observation->numInts) << endl;
				if (startResponse->observation->numDoubles)
					os << "     " << printArray<double>("doubles", stepResponse->observation->doubleArray, stepResponse->observation->numDoubles) << endl;
				if (startResponse->observation->numChars)
					os << "     " << printArray<char>("chars", stepResponse->observation->charArray, stepResponse->observation->numChars) << endl;
				os << "    </Observation>" << endl;

			}
		}
		terminal = stepResponse->terminal;
		os << "   </Trajectory>" << endl;
	}
	reward_t r = RL_return();
	cumulative += r;
	int steps = RL_num_steps();
	steps_total += steps;
	time_t duration = time_in_milli()-start_time;
	total_time += duration;
	term_total += terminal;
	if (output == xml) {
		os << "   <steps>" << steps << "</steps>" << endl
		   << "   <return>" << r << "</return>" << endl
		   << "   <duration>" << duration << "</duration>" << endl;
		if (!terminal)
			os << "   <nonterminal/>" << endl;
		os << "  </Episode>" << endl;
	}
	if (output == csv) {
		os << steps
		   << "\t" << steps_total
           << "\t" << r
           << "\t" << cumulative
		   << "\t" << duration
		   << "\t" << total_time
		   << "\t" << terminal
		   << "\t" << term_total << endl;
	}
}

int main(int argc, char *argv[]) {
	output = xml;
	experiment_verbose = false;

	int num_eps = 10;
	int step_limit = 0;
	const char* out_path = "/dev/stdout";
	const char* exp_id = "";
	int seed = time(0);

	int j=0;
	for (int i=1; i<argc; i++) {
		if (!strcmp(argv[i], "-xml")) {
			output = xml;
			continue;
		}
		if (!strcmp(argv[i], "-csv")) {
			output = csv;
			continue;
		}
		if (!strcmp(argv[i], "-v")) {
			experiment_verbose = true;
			continue;
		}
		if (!strcmp(argv[i], "-seed")) {
			seed = atoi(argv[++i]);
			continue;
		}
		if (j == 0)
			num_eps = atoi(argv[i]);
		if (j == 1)
			step_limit = atoi(argv[i]);
		if (j == 2)
			out_path = argv[i];
		if (j == 3)
			exp_id = argv[i];
		if (j == 4) {
			setenv("RLGLUE_HOST", strtok(argv[i], ":"), 1);
			setenv("RLGLUE_PORT", strtok(0, ":"), 1);
		}
		j++;
	}
	if (j>5) {
		cerr << "Usage: " << argv[0] << " [options] [episodes [max steps [log output [experiment id [host:port]]]]]" << endl;
		cerr << " Options:" << endl;
		cerr << "  -xml, -csv: output format" << endl;
		cerr << "  -seed x: seed the environment and agent with x" << endl;
		cerr << "  -v: verbose - print the entire trajectory (only in xml mode)" << endl;
		return 1;
	}

	if (experiment_verbose && output == csv) {
		cerr << "Cannot run experiment in verbose mode with csv output" << endl;
		return 1;
	}

	ofstream os(out_path);

	time_t start_time = time_in_milli();

	char* task_spec = strdup(RL_init());

	char* agent_id = strdup(RL_agent_message((char*)"id"));
	char* agent_param = strdup(RL_agent_message((char*)"param"));
	char* agent_vers = strdup(RL_agent_message((char*)"version"));
	char* env_id = strdup(RL_env_message((char*)"id"));
	char* env_param = strdup(RL_env_message((char*)"param"));
	char* env_vers = strdup(RL_env_message((char*)"version"));

	{
		ostringstream seed_os;
		seed_os << "seed " << seed;
		char* seed_str = (char*)seed_os.str().c_str();
		RL_agent_message(seed_str);
	}
	{
		ostringstream seed_os;
		seed_os << "seed " << (seed+1);
		char* seed_str = (char*)seed_os.str().c_str();
		RL_env_message(seed_str);
	}

	if (output == xml) {
		if (!exp_id)
			os << "<Experiment>" << endl;
		else
			os << "<Experiment id=\"" << exp_id << "\">" << endl;
		os << " <Task>" << task_spec << "</Task>" << endl
		   << " <Agent name=\"" << agent_id << "\" seed=\"" << seed << "\" version=\"" << agent_vers << "\">" << endl
		   << "  " << agent_param << endl
		   << " </Agent>" << endl
		   << " <Environment name=\"" << env_id << "\" seed=\"" << (seed+1) << "\" version=\"" << env_vers << "\">" << endl
		   << "  " << env_param << endl
		   << " </Environment>" << endl
		   << " <Episodes>" << endl;
	}
	if (output == csv) {

		os << "Experiment ID:\t" << (exp_id?exp_id:"") << endl
		   << "Task:\t" << task_spec << endl
		   << "Agent:\t" << agent_id << endl
		   << "Agent version:\t" << agent_vers << endl
		   << "Agent parameters:\t" << agent_param << endl
		   << "Agent seed:\t" << seed << endl
		   << "Environment:\t" << env_id << endl
		   << "Environment version:\t" << env_vers << endl
		   << "Environment parameters:\t" << env_param << endl
		   << "Environment seed:\t" << (seed+1) << endl
		   << endl
		   << "Steps\tTotalSteps\tReturn\tTotalReturn\tTime\tTotalTime\tTerminal\tTotalTerminal" << endl;
	}



    reward_t total_return = 0;
    int total_steps = 0;
    int total_term = 0;
    time_t total_time = time_in_milli()-start_time;
	for (int i=0; i<num_eps; i++)
		runEpisode(os, i, step_limit, total_steps, total_return, total_time, total_term);
	reward_t avg_return = total_return/num_eps;
	if (output == xml) {
		os << " </Episodes>" << endl
		   << " <averageReturn>" << avg_return << "</averageReturn>" << endl
		   << "</Experiment>" << endl;
	}

	RL_cleanup();
	free(task_spec);
	free(agent_id);
	free(agent_vers);
	free(agent_param);
	free(env_id);
	free(env_param);
	free(env_vers);

	return 0;
}
