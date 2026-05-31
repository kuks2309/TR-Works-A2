string   name           # used as PlayScenario.Goal.scenario_path by main_window
string   description     # may be empty
int32    sub_count       # == len(sub_names)
string[] sub_names       # native list (no comma join)
string   path            # absolute filesystem path (server fills if known)
