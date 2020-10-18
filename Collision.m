classdef Collision < handle
    
    properties

        % Table collision box
        tableC = [0 0 0.55];
        tableS = [2 1.5 1.1];
        
        % Shelf1
        shelf1C = [0.7 0 1.1585];
        shelf1S = [0.2 0.8 0.02];
        
        % Shelf2
        shelf2C = [0.7 0 1.4255];
        shelf2S = [0.2 0.8 0.02];
        
        % Shelf3
        shelf3C = [0.7 0 1.6925];
        shelf3S = [0.2 0.8 0.02];
        
        % Shelf4
        shelf4C = [0.7 0 1.9595];
        shelf4S = [0.2 0.8 0.02];
        
        % Shelf5
        shelf5C = [0.7 0 2.2265];
        shelf5S = [0.2 0.8 0.02];
        
        plot = true;
        
    end
    
    methods(Static)
        
        function self = Collision()
            self.createTableBox(self);
            self.createShelf1Box(self);
            self.createShelf2Box(self);
            self.createShelf3Box(self);
            self.createShelf4Box(self);
            self.createShelf5Box(self);
        end
        
        function createTableBox(self)
            plotOptions.plotFaces = self.plot;
            [v, f, data] = RectangularPrism(self.tableC + self.tableS/2, self.tableC - self.tableS/2, plotOptions);
        end
        
        function createShelf1Box(self)
            plotOptions.plotFaces = self.plot;
            [v, f, data] = RectangularPrism(self.shelf1C + self.shelf1S, self.shelf1C - self.shelf1S, plotOptions);
        end
        
        function createShelf2Box(self)
            plotOptions.plotFaces = self.plot;
            [v, f, data] = RectangularPrism(self.shelf2C + self.shelf2S, self.shelf2C - self.shelf2S, plotOptions);
        end
        
        function createShelf3Box(self)
            plotOptions.plotFaces = self.plot;
            [v, f, data] = RectangularPrism(self.shelf3C + self.shelf3S, self.shelf3C - self.shelf3S, plotOptions);
        end
        
        function createShelf4Box(self)
            plotOptions.plotFaces = self.plot;
            [v, f, data] = RectangularPrism(self.shelf4C + self.shelf4S, self.shelf4C - self.shelf4S, plotOptions);
        end
        
        function createShelf5Box(self)
            plotOptions.plotFaces = self.plot;
            [v, f, data] = RectangularPrism(self.shelf5C + self.shelf5S, self.shelf5C - self.shelf5S, plotOptions);
        end
        
        
        
    end
    
end