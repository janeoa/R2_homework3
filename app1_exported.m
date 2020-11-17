classdef app1_exported < matlab.apps.AppBase

    % Properties that correspond to app components
    properties (Access = public)
        UIFigure                  matlab.ui.Figure
        FPSSliderLabel            matlab.ui.control.Label
        FPSSlider                 matlab.ui.control.Slider
        LeftWheelSliderLabel      matlab.ui.control.Label
        LeftWheelSlider           matlab.ui.control.Slider
        RightWheelSliderLabel     matlab.ui.control.Label
        RightWheelSlider          matlab.ui.control.Slider
        UIAxes                    matlab.ui.control.UIAxes
        runSwitchLabel            matlab.ui.control.Label
        runSwitch                 matlab.ui.control.ToggleSwitch
        TaskSwitchLabel           matlab.ui.control.Label
        TaskSwitch                matlab.ui.control.Switch
        FinishLabel               matlab.ui.control.Label
        startingangleSliderLabel  matlab.ui.control.Label
        startingangleSlider       matlab.ui.control.Slider
    end

    
    properties (Access = private)
        myTimer % Description
        grid % Description
        robot_direction = 0 % Description
        robot_position_vector = [115 106];
        light_position = [910 547];
%         simulinkus = myModel1;
        dimX = 1000;
        dimY = 663;
        sizO = [1000 633];
        simout;
        simout_index;
        no_collider = false;
        init_dir = 30;
    end
    
    
    methods (Access = private)
        
        function timer_callback_fcn(app)
            collision_power = 0.05;
            if(app.no_collider)
               collision_power =0; 
            end
            
            collide(app, [308 264], 60, collision_power, false);
            collide(app, [267 539], 60, collision_power, false);
            collide(app, [684 279], 60, collision_power, false);
            collide(app, [902 100], 60, collision_power, false);
            collide(app, [910 547], 10, collision_power, true);
            
            x = app.robot_position_vector(1);
            y = app.robot_position_vector(2);
            o = app.robot_direction*pi/180;
            dt = round(100/app.FPSSlider.Value)/100 * 2;
            
            theta = app.robot_direction-90;
            RotSen = [cosd(theta) -sind(theta); sind(theta) cosd(theta)];
            LL = RotSen*[-35;65]+app.robot_position_vector';
            RL = RotSen*[ 35;65]+app.robot_position_vector';
            
            dLL = (1-sqrt((LL(1)-app.light_position(1))^2+(LL(2)-app.light_position(2))^2)/sqrt(app.dimX^2+app.dimY^2))^4;
            dRL = (1-sqrt((RL(1)-app.light_position(1))^2+(RL(2)-app.light_position(2))^2)/sqrt(app.dimX^2+app.dimY^2))^4;
            
            CL = checkCollisionSensor([round(LL(1)) round(LL(2))]);
            CR = checkCollisionSensor([round(RL(1)) round(RL(2))]);
            if(app.no_collider)
               CL=0; 
               CR=0;
            end
            
            ML = 0.6*dRL-0.15*CR+0.25;
            MR = 0.6*dLL-0.15*CL+0.25;
            
            ans = [CL CR ML MR]
            
            speed0 = 120;
            
            new = DK([x y], o, speed0*[ML MR], dt);
            x = new(1);
            y = new(2);
            o = new(3);
            
            app.robot_position_vector = checkBorders([x y]);
            app.robot_direction = o*180/pi;
%             LL = app.light_position - 
            
            toShow = boolean(app.grid + putRobot(app.grid, round(app.robot_position_vector), app.robot_direction));
            toShow = 1-flipud(transpose(toShow));
            imshow(toShow, 'Parent',app.UIAxes);
            
            hold(app.UIAxes , "on");
%             disp(app.grid(230,100))
            plot(app.UIAxes, LL(1), app.dimY-LL(2),'.','MarkerSize',30)
            plot(app.UIAxes, RL(1), app.dimY-RL(2),'.','MarkerSize',30)
%             disp(app.grid(round(LL(1),round(LL(1)))));
%             plot(app.UIAxes, app.robot_position_vector+LL,'.','MarkerSize',10)
%             plot(app.UIAxes, app.robot_position_vector+RL, '.','MarkerSize',10)
%             plot(app.UIAxes, app.robot_position_vector,'.','MarkerSize',10);
            hold(app.UIAxes , "off");
        end
        
        
        function task1loop(app)
            x = app.robot_position_vector(1);
            y = app.robot_position_vector(2);
            o = app.robot_direction*pi/180;
            dt = round(100/app.FPSSlider.Value)/100 * 2;
            
            speedLeft = app.LeftWheelSlider.Value;
            speedRight = app.RightWheelSlider.Value;
            
%             fprintf("\npos [%2.1f %2.1f] dir [%2.1f] dt [%2.2f]\n\tspd [%2.2f %2.2f]", ...
%                 x,y,o,dt, speedLeft, speedRight)
            
            new = DK([x y], o, [speedLeft speedRight], dt);
            x = new(1);
            y = new(2);
            o = new(3);
            
            app.robot_position_vector = checkBorders([x y]);
            app.robot_direction = o*180/pi;
            
            toShow = boolean(app.grid + putRobot(app.grid, round(app.robot_position_vector), app.robot_direction));
            toShow = 1-flipud(transpose(toShow));
            imshow(toShow, 'Parent',app.UIAxes);
        end        
        
        
        function collide(app, C, R, collider_power, isLight)
            if(checkCollision(app.robot_position_vector, 78, C, R))
                if(isLight)
                    delete(app.myTimer)
                    app.FinishLabel.Visible = true;
                end
                disp("COLLISION!")
                epta = round(collider_power*(app.robot_position_vector - C))
                app.robot_position_vector = app.robot_position_vector + round(collider_power*(app.robot_position_vector - C));
            end
        end
    end
  

    % Callbacks that handle component events
    methods (Access = private)

        % Code that executes after component creation
        function startupFcn(app)
            addpath("helper/");
            
            app.grid = boolean(zeros(app.dimX, app.dimY)); % in mm
            app.grid = drawSq(app.grid, 1,1,app.dimX, app.dimY,1);
            
            app.robot_position_vector = [500 300];
            
            toShow = boolean(app.grid + putRobot(app.grid, app.robot_position_vector, round(app.robot_direction)));
            
            toShow = 1-flipud(transpose(toShow));
            imshow(toShow, 'Parent',app.UIAxes);
            
            app.myTimer = timer('Period', round(100/app.FPSSlider.Value)/100, 'ExecutionMode', 'fixedSpacing','StartDelay',3);
            fprintf('\ntimer is set to %f\n', round(100/app.FPSSlider.Value)/100);
            app.myTimer.TimerFcn = @(~,~)app.task1loop;
            start(app.myTimer);
        end

        % Value changed function: runSwitch
        function runSwitchValueChanged(app, event)
            app.init_dir = app.startingangleSlider.Value;
            app.robot_direction = app.init_dir;
            value = app.runSwitch.Value;
            
            if(value=="On")
                app.myTimer = timer('Period', round(100/app.FPSSlider.Value)/100, 'ExecutionMode', 'fixedSpacing');
                disp('timer is set to' + round(100/app.FPSSlider.Value)/100);
                app.myTimer.TimerFcn = @(~,~)app.timer_callback_fcn;
                
                start(app.myTimer)
            else
                disp('deleting');
                delete(app.myTimer)
            end
            
            
        end

        % Value changed function: FPSSlider
        function fpsChange(app, event)
            disp('chaging fps');
            delete(app.myTimer)
            app.runSwitch.Value="Off";
            
            if(app.TaskSwitch.Value == "1")
                app.myTimer = timer('Period', round(100/app.FPSSlider.Value)/100, 'ExecutionMode', 'fixedSpacing','StartDelay',1);
                fprintf('\ntimer is set to %f\n', round(100/app.FPSSlider.Value)/100);
                app.myTimer.TimerFcn = @(~,~)app.task1loop;
                start(app.myTimer);
            end
        end

        % Value changed function: TaskSwitch
        function changeTask(app, event)
            delete(app.myTimer)
            
            app.TaskSwitch.Enable = false;
            app.dimY = 663; app.dimX = 1000;
            app.grid = boolean(zeros(app.dimX,app.dimY));
            app.LeftWheelSlider.Visible = false;
            app.RightWheelSlider.Visible = false;
            app.LeftWheelSlider.Enable = false;
            app.RightWheelSlider.Enable= false;
            app.LeftWheelSliderLabel.Visible = false;
            app.RightWheelSliderLabel.Visible= false;
            
            app.startingangleSlider.Visible = true;
            app.startingangleSliderLabel.Visible=true;
            app.runSwitch.Enable = true;
            
            
            if(~app.no_collider)
                app.grid = boolean(app.grid + putObstacle(app.dimX, app.dimY, 308, 264, 60));
                app.grid = boolean(app.grid + putObstacle(app.dimX, app.dimY, 267, 539, 60));
                app.grid = boolean(app.grid + putObstacle(app.dimX, app.dimY, 684, 279, 60));
                app.grid = boolean(app.grid + putObstacle(app.dimX, app.dimY, 902, 100, 60));
            end
            app.grid = boolean(app.grid + putObstacle(app.dimX, app.dimY, 910, 547, 10));
            
            %910 547;
            stgz = 200;
            [gray,colord] = drawGrad(app.dimX, app.dimY, app.light_position(1), app.light_position(2), stgz);
            

            toShow = 1-flipud(transpose(app.grid));
            
            
            % bg image
            [img, ~, alphachannel] = imread("map.png");
            image(img, 'AlphaData', 0.3*alphachannel, 'Parent', app.UIAxes);
            
            hold(app.UIAxes , "on");
            
            % the light suorce
            imgz = imshow(colord, 'Parent',app.UIAxes);
            set(imgz, 'AlphaData', stgz-gray);
            
            obsticles = imshow(toShow, 'Parent',app.UIAxes);
            set(obsticles, 'AlphaData', 1-toShow);
            hold(app.UIAxes , "off");
                           
            app.robot_position_vector = [115 106];
        end
    end

    % Component initialization
    methods (Access = private)

        % Create UIFigure and components
        function createComponents(app)

            % Create UIFigure and hide until all components are created
            app.UIFigure = uifigure('Visible', 'off');
            app.UIFigure.Position = [100 100 640 480];
            app.UIFigure.Name = 'MATLAB App';

            % Create FPSSliderLabel
            app.FPSSliderLabel = uilabel(app.UIFigure);
            app.FPSSliderLabel.HorizontalAlignment = 'right';
            app.FPSSliderLabel.Position = [41 402 28 22];
            app.FPSSliderLabel.Text = 'FPS';

            % Create FPSSlider
            app.FPSSlider = uislider(app.UIFigure);
            app.FPSSlider.Limits = [5 40];
            app.FPSSlider.MajorTicks = [5 10 15 20 25 30 35 40];
            app.FPSSlider.ValueChangedFcn = createCallbackFcn(app, @fpsChange, true);
            app.FPSSlider.MinorTicks = [5 10 15 20 25 30 35 40];
            app.FPSSlider.Position = [90 411 218 3];
            app.FPSSlider.Value = 15;

            % Create LeftWheelSliderLabel
            app.LeftWheelSliderLabel = uilabel(app.UIFigure);
            app.LeftWheelSliderLabel.HorizontalAlignment = 'right';
            app.LeftWheelSliderLabel.Position = [377 443 63 22];
            app.LeftWheelSliderLabel.Text = 'Left Wheel';

            % Create LeftWheelSlider
            app.LeftWheelSlider = uislider(app.UIFigure);
            app.LeftWheelSlider.Limits = [-100 100];
            app.LeftWheelSlider.MinorTicks = [-100 -90 -80 -70 -60 -50 -40 -30 -20 -10 0 10 20 30 40 50 60 70 80 90 100];
            app.LeftWheelSlider.Position = [461 452 150 3];
            app.LeftWheelSlider.Value = 50;

            % Create RightWheelSliderLabel
            app.RightWheelSliderLabel = uilabel(app.UIFigure);
            app.RightWheelSliderLabel.HorizontalAlignment = 'right';
            app.RightWheelSliderLabel.Position = [370 392 70 22];
            app.RightWheelSliderLabel.Text = 'Right Wheel';

            % Create RightWheelSlider
            app.RightWheelSlider = uislider(app.UIFigure);
            app.RightWheelSlider.Limits = [-100 100];
            app.RightWheelSlider.MinorTicks = [-100 -90 -80 -70 -60 -50 -40 -30 -20 -10 0 10 20 30 40 50 60 70 80 90 100];
            app.RightWheelSlider.Position = [461 401 150 3];

            % Create UIAxes
            app.UIAxes = uiaxes(app.UIFigure);
            title(app.UIAxes, 'Asset Malik')
            xlabel(app.UIAxes, 'X')
            ylabel(app.UIAxes, 'Y')
            app.UIAxes.Position = [22 12 600 358];

            % Create runSwitchLabel
            app.runSwitchLabel = uilabel(app.UIFigure);
            app.runSwitchLabel.HorizontalAlignment = 'center';
            app.runSwitchLabel.Enable = 'off';
            app.runSwitchLabel.Position = [218 442 25 22];
            app.runSwitchLabel.Text = 'run';

            % Create runSwitch
            app.runSwitch = uiswitch(app.UIFigure, 'toggle');
            app.runSwitch.Orientation = 'horizontal';
            app.runSwitch.ValueChangedFcn = createCallbackFcn(app, @runSwitchValueChanged, true);
            app.runSwitch.Enable = 'off';
            app.runSwitch.Position = [277 443 45 20];

            % Create TaskSwitchLabel
            app.TaskSwitchLabel = uilabel(app.UIFigure);
            app.TaskSwitchLabel.HorizontalAlignment = 'center';
            app.TaskSwitchLabel.Position = [41 442 30 22];
            app.TaskSwitchLabel.Text = 'Task';

            % Create TaskSwitch
            app.TaskSwitch = uiswitch(app.UIFigure, 'slider');
            app.TaskSwitch.Items = {'1', '2'};
            app.TaskSwitch.ValueChangedFcn = createCallbackFcn(app, @changeTask, true);
            app.TaskSwitch.Position = [96 443 45 20];
            app.TaskSwitch.Value = '1';

            % Create FinishLabel
            app.FinishLabel = uilabel(app.UIFigure);
            app.FinishLabel.FontSize = 100;
            app.FinishLabel.FontColor = [0.4667 0.6745 0.1882];
            app.FinishLabel.Visible = 'off';
            app.FinishLabel.Position = [194 99 298 130];
            app.FinishLabel.Text = 'Finish!';

            % Create startingangleSliderLabel
            app.startingangleSliderLabel = uilabel(app.UIFigure);
            app.startingangleSliderLabel.HorizontalAlignment = 'right';
            app.startingangleSliderLabel.Visible = 'off';
            app.startingangleSliderLabel.Position = [368 422 78 22];
            app.startingangleSliderLabel.Text = 'starting angle';

            % Create startingangleSlider
            app.startingangleSlider = uislider(app.UIFigure);
            app.startingangleSlider.Limits = [0 90];
            app.startingangleSlider.Visible = 'off';
            app.startingangleSlider.Position = [467 431 150 3];
            app.startingangleSlider.Value = 30;

            % Show the figure after all components are created
            app.UIFigure.Visible = 'on';
        end
    end

    % App creation and deletion
    methods (Access = public)

        % Construct app
        function app = app1_exported

            % Create UIFigure and components
            createComponents(app)

            % Register the app with App Designer
            registerApp(app, app.UIFigure)

            % Execute the startup function
            runStartupFcn(app, @startupFcn)

            if nargout == 0
                clear app
            end
        end

        % Code that executes before app deletion
        function delete(app)

            % Delete UIFigure when app is deleted
            delete(app.UIFigure)
        end
    end
end