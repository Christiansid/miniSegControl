% Setup Arduino
setup_arduino

% Add paths to RASPlib
addpath([fileparts(mfilename('fullpath')) '/RASPlib'],...
		[fileparts(mfilename('fullpath')) '/RASPlib/src'],...
		[fileparts(mfilename('fullpath')) '/RASPlib/include'],...
		[fileparts(mfilename('fullpath')) '/RASPlib/examples'],...
		[fileparts(mfilename('fullpath')) '/RASPlib/blocks'])
 
