function fig = SixDofAnimationTwin(varargin)

    %% Create local variables
    % Required arguments
    p = varargin{1};                % position of body
    R = varargin{2};                % rotation matrix of body
    p1 = varargin{3};                % position of body
    R1 = varargin{4};                % rotation matrix of body
    [numSamples dummy] = size(p);

    % Default values of optional arguments
    SamplePlotFreq = 1;
    Trail = 'Off';
    LimitRatio = 1+2;
    Position = [];
    FullScreen = false;
    View = [30 20];
    AxisLength = 1;
    ShowArrowHead = 'on';
    Xlabel = 'X';
    Ylabel = 'Y';
    Zlabel = 'Z';
    Title = '6DOF Animation';
    ShowLegend = true;
    CreateAVI = false;
    AVIfileName = '6DOF Animation';
    AVIfileNameEnum = true;
    AVIfps = 30;

    for i = 5:2:nargin
        if  strcmp(varargin{i}, 'SamplePlotFreq'), SamplePlotFreq = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Trail')
            Trail = varargin{i+1};
            if(~strcmp(Trail, 'Off') && ~strcmp(Trail, 'DotsOnly') && ~strcmp(Trail, 'All'))
                error('Invalid argument.  Trail must be ''Off'', ''DotsOnly'' or ''All''.');
            end
        elseif  strcmp(varargin{i}, 'LimitRatio'), LimitRatio = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Position'), Position = varargin{i+1};
        elseif  strcmp(varargin{i}, 'FullScreen'), FullScreen = varargin{i+1};
        elseif  strcmp(varargin{i}, 'View'), View = varargin{i+1};
        elseif  strcmp(varargin{i}, 'AxisLength'), AxisLength = varargin{i+1};
        elseif  strcmp(varargin{i}, 'ShowArrowHead'), ShowArrowHead = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Xlabel'), Xlabel = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Ylabel'), Ylabel = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Zlabel'), Zlabel = varargin{i+1};
        elseif  strcmp(varargin{i}, 'Title'), Title = varargin{i+1};
        elseif  strcmp(varargin{i}, 'ShowLegend'), ShowLegend = varargin{i+1};
        elseif  strcmp(varargin{i}, 'CreateAVI'), CreateAVI = varargin{i+1};
        elseif  strcmp(varargin{i}, 'AVIfileName'), AVIfileName = varargin{i+1};
        elseif  strcmp(varargin{i}, 'AVIfileNameEnum'), AVIfileNameEnum = varargin{i+1};
        elseif  strcmp(varargin{i}, 'AVIfps'), AVIfps = varargin{i+1};
        else error('Invalid argument.');
        end
    end;

    %% Reduce data to samples to plot only

    p = p(1:SamplePlotFreq:numSamples, :);
    R = R(:, :, 1:SamplePlotFreq:numSamples) * AxisLength;
    p1 = p1(1:SamplePlotFreq:numSamples, :);
    R1 = R1(:, :, 1:SamplePlotFreq:numSamples) * AxisLength;
    if(numel(View) > 2)
        View = View(1:SamplePlotFreq:numSamples, :);
    end
    [numPlotSamples dummy] = size(p);

    

    %% Setup figure and plot

    % Create figure
    fig = figure('NumberTitle', 'off', 'Name', '6DOF Animation');
    if(FullScreen)
        screenSize = get(0, 'ScreenSize');
        set(fig, 'Position', [0 0 screenSize(3) screenSize(4)]);
    elseif(~isempty(Position))
        set(fig, 'Position', Position);
    end
    set(gca, 'drawmode', 'fast');
    lighting phong;
    set(gcf, 'Renderer', 'zbuffer');
    hold on;
    axis equal;
    grid on;
    view(View(1, 1), View(1, 2));
    title(i);
    xlabel(Xlabel);
    ylabel(Ylabel);
    zlabel(Zlabel);

    % Create plot data arrays
    if(strcmp(Trail, 'DotsOnly') || strcmp(Trail, 'All'))
        x = zeros(numPlotSamples, 1);
        y = zeros(numPlotSamples, 1);
        z = zeros(numPlotSamples, 1);

        x1 = zeros(numPlotSamples, 1);
        y1 = zeros(numPlotSamples, 1);
        z1 = zeros(numPlotSamples, 1);
    end
    if(strcmp(Trail, 'All'))
        ox = zeros(numPlotSamples, 1);
        oy = zeros(numPlotSamples, 1);
        oz = zeros(numPlotSamples, 1);
        ux = zeros(numPlotSamples, 1);
        vx = zeros(numPlotSamples, 1);
        wx = zeros(numPlotSamples, 1);
        uy = zeros(numPlotSamples, 1);
        vy = zeros(numPlotSamples, 1);
        wy = zeros(numPlotSamples, 1);
        uz = zeros(numPlotSamples, 1);
        vz = zeros(numPlotSamples, 1);
        wz = zeros(numPlotSamples, 1);

        ox1 = zeros(numPlotSamples, 1);
        oy1 = zeros(numPlotSamples, 1);
        oz1 = zeros(numPlotSamples, 1);
        ux1 = zeros(numPlotSamples, 1);
        vx1 = zeros(numPlotSamples, 1);
        wx1 = zeros(numPlotSamples, 1);
        uy1 = zeros(numPlotSamples, 1);
        vy1 = zeros(numPlotSamples, 1);
        wy1 = zeros(numPlotSamples, 1);
        uz1 = zeros(numPlotSamples, 1);
        vz1 = zeros(numPlotSamples, 1);
        wz1 = zeros(numPlotSamples, 1);
    end
    x(1) = p(1,1);
    y(1) = p(1,2);
    z(1) = p(1,3);
    ox(1) = x(1);
    oy(1) = y(1);
    oz(1) = z(1);
    ux(1) = R(1,1,1:1);
    vx(1) = R(2,1,1:1);
    wx(1) = R(3,1,1:1);
    uy(1) = R(1,2,1:1);
    vy(1) = R(2,2,1:1);
    wy(1) = R(3,2,1:1);
    uz(1) = R(1,3,1:1);
    vz(1) = R(2,3,1:1);
    wz(1) = R(3,3,1:1);

    x1(1) = p1(1,1);
    y1(1) = p1(1,2);
    z1(1) = p1(1,3);
    ox1(1) = x1(1);
    oy1(1) = y1(1);
    oz1(1) = z1(1);
    ux1(1) = R1(1,1,1:1);
    vx1(1) = R1(2,1,1:1);
    wx1(1) = R1(3,1,1:1);
    uy1(1) = R1(1,2,1:1);
    vy1(1) = R1(2,2,1:1);
    wy1(1) = R1(3,2,1:1);
    uz1(1) = R1(1,3,1:1);
    vz1(1) = R1(2,3,1:1);
    wz1(1) = R1(3,3,1:1);

    % Create graphics handles
    orgHandle = plot3(x, y, z, 'k.');
    orgHandle1 = plot3(x1, y1, z1, 'k.');

    if(ShowArrowHead)
        ShowArrowHeadStr = 'on';
    else
        ShowArrowHeadStr = 'off';
    end
    quivXhandle = quiver3(ox, oy, oz, ux, vx, wx,  'r', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivYhandle = quiver3(ox, oy, oz, uy, vy, wy,  'g', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivZhandle = quiver3(ox, oy, oz, uz, vz, wz,  'b', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');

    quivXhandle1 = quiver3(ox1, oy1, oz1, ux1, vx1, wx1,  'r', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivYhandle1 = quiver3(ox1, oy1, oz1, uy1, vy1, wy1,  'g', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');
    quivZhandle1 = quiver3(ox1, oy1, oz1, uz1, vz1, wz1,  'b', 'ShowArrowHead', ShowArrowHeadStr, 'MaxHeadSize', 0.999999, 'AutoScale', 'off');


    % Create legend
    if(ShowLegend)
        legend('Origin', 'X', 'Y', 'Z');
    end
    
    % Set initial limits
    Xlim = [x(1)-AxisLength x(1)+AxisLength] * LimitRatio;
    Ylim = [y(1)-AxisLength y(1)+AxisLength] * LimitRatio;
    Zlim = [z(1)-AxisLength z(1)+AxisLength] * LimitRatio;
    
    Xlim1 = [x1(1)-AxisLength x1(1)+AxisLength] * LimitRatio;
    Ylim1 = [y1(1)-AxisLength y1(1)+AxisLength] * LimitRatio;
    Zlim1 = [z1(1)-AxisLength z1(1)+AxisLength] * LimitRatio;

    Xlim=[min(Xlim(1),Xlim1(1)),max(Xlim(2),Xlim1(2))];
    Ylim=[min(Ylim(1),Ylim1(1)),max(Ylim(2),Ylim1(2))];
    Zlim=[min(Zlim(1),Zlim1(1)),max(Zlim(2),Zlim1(2))];

    set(gca, 'Xlim', Xlim, 'Ylim', Ylim, 'Zlim', Zlim);
    
    % Set initial view
    view(View(1, :));


    %% Plot one sample at a time

    for i = 1:numPlotSamples

        % Update graph title
        if(strcmp(Title, ''))
            titleText = sprintf('Sample %i of %i', 1+((i-1)*SamplePlotFreq), numSamples);
        else
            titleText = strcat(Title, ' (', sprintf('Sample %i of %i', 1+((i-1)*SamplePlotFreq), numSamples), ')');
        end
        title(titleText);

        % Plot body x y z axes
        if(strcmp(Trail, 'DotsOnly') || strcmp(Trail, 'All'))
            x(1:i) = p(1:i,1);
            y(1:i) = p(1:i,2);
            z(1:i) = p(1:i,3);

            x1(1:i) = p1(1:i,1);
            y1(1:i) = p1(1:i,2);
            z1(1:i) = p1(1:i,3);       
        else
            x = p(i,1);
            y = p(i,2);
            z = p(i,3);

            x1 = p1(i,1);
            y1 = p1(i,2);
            z1 = p1(i,3);
        end
        if(strcmp(Trail, 'All'))
            ox(1:i) = p(1:i,1);
            oy(1:i) = p(1:i,2);
            oz(1:i) = p(1:i,3);
            ux(1:i) = R(1,1,1:i);
            vx(1:i) = R(2,1,1:i);
            wx(1:i) = R(3,1,1:i);
            uy(1:i) = R(1,2,1:i);
            vy(1:i) = R(2,2,1:i);
            wy(1:i) = R(3,2,1:i);
            uz(1:i) = R(1,3,1:i);
            vz(1:i) = R(2,3,1:i);
            wz(1:i) = R(3,3,1:i);

            ox1(1:i) = p1(1:i,1);
            oy1(1:i) = p1(1:i,2);
            oz1(1:i) = p1(1:i,3);
            ux1(1:i) = R1(1,1,1:i);
            vx1(1:i) = R1(2,1,1:i);
            wx1(1:i) = R1(3,1,1:i);
            uy1(1:i) = R1(1,2,1:i);
            vy1(1:i) = R1(2,2,1:i);
            wy1(1:i) = R1(3,2,1:i);
            uz1(1:i) = R1(1,3,1:i);
            vz1(1:i) = R1(2,3,1:i);
            wz1(1:i) = R1(3,3,1:i);
        else
            ox = p(i,1);
            oy = p(i,2);
            oz = p(i,3);
            ux = R(1,1,i);
            vx = R(2,1,i);
            wx = R(3,1,i);
            uy = R(1,2,i);
            vy = R(2,2,i);
            wy = R(3,2,i);
            uz = R(1,3,i);
            vz = R(2,3,i);
            wz = R(3,3,i);

            ox1 = p1(i,1);
            oy1 = p1(i,2);
            oz1 = p1(i,3);
            ux1 = R1(1,1,i);
            vx1 = R1(2,1,i);
            wx1 = R1(3,1,i);
            uy1 = R1(1,2,i);
            vy1 = R1(2,2,i);
            wy1 = R1(3,2,i);
            uz1 = R1(1,3,i);
            vz1 = R1(2,3,i);
            wz1 = R1(3,3,i);

        end

        hold on

        set(orgHandle, 'xdata', x, 'ydata', y, 'zdata', z);
        set(quivXhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', ux, 'vdata', vx, 'wdata', wx);
        set(quivYhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uy, 'vdata', vy, 'wdata', wy);
        set(quivZhandle, 'xdata', ox, 'ydata', oy, 'zdata', oz,'udata', uz, 'vdata', vz, 'wdata', wz);

        set(orgHandle1, 'xdata', x1, 'ydata', y1, 'zdata', z1);
        set(quivXhandle1, 'xdata', ox1, 'ydata', oy1, 'zdata', oz1,'udata', ux1, 'vdata', vx1, 'wdata', wx1);
        set(quivYhandle1, 'xdata', ox1, 'ydata', oy1, 'zdata', oz1,'udata', uy1, 'vdata', vy1, 'wdata', wy1);
        set(quivZhandle1, 'xdata', ox1, 'ydata', oy1, 'zdata', oz1,'udata', uz1, 'vdata', vz1, 'wdata', wz1);


        % Adjust axes for snug fit and draw
        axisLimChanged = false;
        if((p(i,1) - AxisLength) < Xlim(1)||(p1(i,1) - AxisLength) < Xlim(1))
            Xlim(1) = min(p(i,1) - LimitRatio*AxisLength,p1(i,1) - AxisLength); axisLimChanged = true; 
        end
        if((p(i,2) - AxisLength) < Ylim(1)||(p1(i,2) - AxisLength) < Ylim(1))
            Ylim(1) = min(p(i,2) - LimitRatio*AxisLength,p1(i,2) - LimitRatio*AxisLength); axisLimChanged = true;
        end
        if((p(i,3) - AxisLength) < Zlim(1)||(p1(i,3) - AxisLength) < Zlim(1))
            Zlim(1) = min(p(i,3) - LimitRatio*AxisLength,p1(i,3) - LimitRatio*AxisLength); axisLimChanged = true; 
        end
        if((p(i,1) + AxisLength) > Xlim(2)||(p1(i,1) + AxisLength) > Xlim(2))
            Xlim(2) = max(p(i,1) + LimitRatio*AxisLength,p1(i,1) + LimitRatio*AxisLength); axisLimChanged = true;
        end
        if((p(i,2) + AxisLength) > Ylim(2)||(p1(i,2) + AxisLength) > Ylim(2))
            Ylim(2) = max(p(i,2) + LimitRatio*AxisLength,p1(i,2) + LimitRatio*AxisLength); axisLimChanged = true;
        end
        if((p(i,3) + AxisLength) > Zlim(2)||(p1(i,3) + AxisLength) > Zlim(2))
            Zlim(2) = max(p(i,3) + LimitRatio*AxisLength,p1(i,3) + LimitRatio*AxisLength); axisLimChanged = true;
        end
        if(axisLimChanged)
            set(gca, 'Xlim', Xlim, 'Ylim', Ylim, 'Zlim', Zlim); 
        end
        drawnow;

        % Adjust view
        if(numel(View) > 2)
            view(View(i, :));
        end

  

    end

    hold off;


end