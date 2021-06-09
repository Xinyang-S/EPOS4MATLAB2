classdef ColorSelector < matlab.ui.componentcontainer.ComponentContainer    
    % UI component to select colors

    % Public properties
    properties
        Value {validateattributes(Value, ...
            {'double'},{'<=',1,'>=',0,'size',[1 3]})} = [1 0 0]; 
    end

    % Events
    events (HasCallbackProperty, NotifyAccess = protected) 
        ValueChanged % ValueChangedFcn will be the generated callback property 
    end
    
    % Private properties
    properties (Access = private, Transient, NonCopyable) 
        Grid matlab.ui.container.GridLayout 
        Button matlab.ui.control.Button 
        EditField matlab.ui.control.EditField 
    end

    methods (Access = protected) 
        function setup(obj) 
            % Grid layout to manage building blocks 
            obj.Grid = uigridlayout(obj,[1 2],'ColumnWidth',{'1x',22}, ... 
                'RowHeight',{'fit'},'ColumnSpacing',2,'Padding',2);              

            % Edit field for value display and button to launch uisetcolor 
            obj.EditField = uieditfield(obj.Grid,'Editable',false, ... 
                'HorizontalAlignment','center'); 
            obj.Button = uibutton(obj.Grid,'Text',char(9998), ... 
                'ButtonPushedFcn',@(o,e) obj.getColorFromUser()); 

        end 
        function update(obj)     
            % Update edit field and button colors 
            set([obj.EditField obj.Button], 'BackgroundColor', obj.Value, ... 
                'FontColor', obj.getContrastingColor(obj.Value));  

            % Update the display text 
            obj.EditField.Value = num2str(obj.Value, '%0.2g ');            
        end 
    end

    methods (Access = private) 
        function getColorFromUser(obj) 
            c = uisetcolor(obj.Value); 
            if (isscalar(c) && (c == 0)) 
                return; 
            end 
            
            % Update the Value property 
            obj.Value = c;              

            % Execute user callbacks and listeners
            notify(obj,'ValueChanged'); 
        end         
        function contrastColor = getContrastingColor(~,color) 
            % Calculate opposite color 
            c = color * 255; 
            contrastColor = [1 1 1]; 
            if (c(1)*.299 + c(2)*.587 + c(3)*.114) > 186 
                contrastColor = [0 0 0]; 
            end 
        end 
    end 
end