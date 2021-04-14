classdef soHCSR04Sonar < matlab.System & coder.ExternalDependency
    % soHCSR04Sonar sonar distance sensor.  To use with 2015b or later try these workaround instructions (at your own risk).  1. Temporary remove the complete line: <sourcefile>----  Tone.cpp<sourcefile> in ArduinoMega2560_attributes.xml which is located in C:\ProgramData\MATLAB\SupportPackages\R2016b\toolbox\target\supportpackages\arduinotarget\registry\attributes.  (Back up this file before making any changes - do not put any files in thie directory)  2.) run the following 3 commands in the matlab prompt: clear classes, rehash toolboxcache, sl_refresh_customizations  3.) open your Simulink model (in this case "sonar.slx"). Go to "Configuration Parameters --> Hardware Implementation " and reselect the "Hardware board" to "Arduino Mega".  You MUST do this. 4.) add the line you removed from  ArduinoMega2560_attributes.xml back (or restore the backup file).  This excludes Tone.cpp, but keeps the file content intact there so it will compile.  May/May not work....
    % 
    %
    
    %#codegen
    %#ok<*EMCA>
    
    properties (Nontunable)
		TrigPin = 7; % Trigger Pin
		EchoPin = 8; % Echo Pin
    end
	
	    properties (Constant, Hidden)
        % AvailablePin specifies the range of values allowed for Pin. You
        % can customize the AvailablePin for a particular board. For
        % example, use AvailablePin = 2:13 for Arduino Uno.
        AvailablePin = 0:53;
    end
	
	
    properties (Hidden,Transient,Constant)

        
    end

	
	
	
	
    properties (Hidden)
%         % keeps track of the selected Potentiometer
%         potNum = 0;
%         % simSampleNum - tracks which sample we are on in a simulation
%         simSampleNum = 0;
    end
    
    methods
	    % Constructor
        function obj = soHCSR04Sonar(varargin)
            coder.allowpcode('plain');
            
            % Support name-value pair arguments when constructing the object.
            setProperties(obj,nargin,varargin{:});
        end
        
        function set.TrigPin(obj,value)
            coder.extrinsic('sprintf') % Do not generate code for sprintf
            validateattributes(value,...
                {'numeric'},...
                {'real', 'positive', 'integer','scalar'},...
                '', ...
                'Pin');
            assert(any(value == obj.AvailablePin), ...
                'Invalid value for Pin. Pin must be one of the following: %s', ...
                sprintf('%d ', obj.AvailablePin));
            obj.TrigPin = value;
        end
		
		 function set.EchoPin(obj,value)
            coder.extrinsic('sprintf') % Do not generate code for sprintf
            validateattributes(value,...
                {'numeric'},...
                {'real', 'positive', 'integer','scalar'},...
                '', ...
                'Pin');
            assert(any(value == obj.AvailablePin), ...
                'Invalid value for Pin. Pin must be one of the following: %s', ...
                sprintf('%d ', obj.AvailablePin));
            obj.EchoPin = value;
        end
		
		
    end

	
	
	
	
	
    methods (Access = protected)
        function setupImpl(obj)
            if coder.target('Rtw')% done only for code gen
                coder.cinclude('HCSR04wrapper.h');
                % initialize the potentiometer
                coder.ceval('HCSR04Sonar_Init', obj.TrigPin, obj.EchoPin);
				% coder.ceval('NewPing sonar(7,8,200);');
            elseif ( coder.target('Sfun') )
                %
            end
        end

        function [d_cm] = stepImpl(obj)
            % initialize output to a single (float) with the value zero
            out = single(zeros(1,1));
            if coder.target('Rtw')% done only for code gen
                coder.cinclude('HCSR04wrapper.h');
                % get the current value of the sensor
                coder.ceval('HCSR04Sonar_Read', coder.wref(out));
            elseif ( coder.target('Sfun') )
                %
            end
            % pull the data appart
            d_cm = out(1);
        end

        function releaseImpl(obj)
            if coder.target('Rtw')% done only for code gen
                %
            elseif ( coder.target('Sfun') )
                %
            end
        end
    end
    
    methods (Static, Access=protected)
        function simMode = getSimulateUsingImpl(~)
            simMode = 'Interpreted execution';
        end
        
        function isVisible = showSimulateUsingImpl
            isVisible = false;
        end
    end

    methods (Static)
        function name = getDescriptiveName()
            name = 'Sonar';
        end
        
        function b = isSupportedContext(context)
            b = context.isCodeGenTarget('rtw');
        end
        
        % Update the build-time buildInfo
        function updateBuildInfo(buildInfo, context)
            if context.isCodeGenTarget('rtw')                
                % Add include paths and source files for code generation
                                
                % determine path to arduino IDE
                try codertarget.target.isCoderTarget(buildInfo.ModelName)
                    % we are in 15b
                    [~, hardwaredir] = codertarget.arduinobase.internal.getArduinoIDERoot('hardware');
                    librarydir = fullfile(hardwaredir, 'arduino', 'avr' , 'libraries');
                catch me
                    % we are pre 15b
                    [~, hardwaredir] = realtime.internal.getArduinoIDERoot('hardware');
                    librarydir = fullfile(hardwaredir, '..', 'libraries');
                 end
                
                % get current directory path
                src_dir = mfilename('fullpath');
                [current_dir] = fileparts(src_dir);
                
                % add the include paths
                buildInfo.addIncludePaths(fullfile(librarydir, 'Wire'));
                buildInfo.addIncludePaths(fullfile(librarydir, 'Wire','utility'));
                buildInfo.addIncludePaths(fullfile(current_dir,'..','include'));
                
                % add the source paths
                srcPaths = {...
                    fullfile(librarydir, 'Wire'), ...
                    fullfile(librarydir, 'Wire', 'utility'),...
                    fullfile(current_dir,'..','src')};
                buildInfo.addSourcePaths(srcPaths);
                
                % add the source files
                srcFiles = {'NewPing.cpp', 'HCSR04wrapper.cpp'};
                buildInfo.addSourceFiles(srcFiles);
                
            end
        end
    end
end
