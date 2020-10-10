workspace = [-1 1 -1 1 0 3];

name = ['KinovaGen3_',datestr(now,'yyyymmddTHHMMSSFFF')];

L(1) = Link('d', 0.1564, 'a', 0, 'alpha', 0, 'qlim', [deg2rad(-360), deg2rad(360)]);
L(2) = Link('d',0.2848,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L(3) = Link('d',0.0054,'a',0.41,'alpha',-pi,'offset',-pi/2,'qlim',[deg2rad(-90),deg2rad(90)]);
L(4) = Link('d',0.0064,'a',0.2084,'alpha',pi,'offset',0,'qlim',[deg2rad(-170),deg2rad(170)]);
L(5) = Link('d',0.0,'a',0,'alpha',-pi/2,'offset',-pi/2,'qlim',[deg2rad(-360),deg2rad(360)]);
L(6) = Link('d',0.1059+.1059,'a',0,'alpha',-pi/2,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);
L(7) = Link('d',0,'a',0,'alpha',0,'offset',0,'qlim',[deg2rad(-360),deg2rad(360)]);

test = SerialLink(L,'name',name);

test.plot(zeros(1, test.n), 'workspace', workspace, 'scale', 0.1);

test.teach;