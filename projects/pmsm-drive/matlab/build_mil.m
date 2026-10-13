function build_mil
% Build a visible Simulink dq plant and cross-check its state with C and ode45.
folder = fileparts(mfilename('fullpath'));
rootFolder = fileparts(folder);
buildFolder = fullfile(rootFolder,'build','matlab');
if ~isfolder(buildFolder), mkdir(buildFolder); end
Simulink.fileGenControl('set','CacheFolder',fullfile(buildFolder,'cache'), ...
    'CodeGenFolder',fullfile(buildFolder,'codegen'),'createDir',true);
model = 'PMSM_DQ_MIL';
if bdIsLoaded(model), close_system(model,0); end
new_system(model);
set_param(model,'Solver','ode4','SolverType','Fixed-step','FixedStep','0.0000125', ...
    'StopTime','0.1','ReturnWorkspaceOutputs','on');
add_block('simulink/Sources/Constant',[model '/vd'],'Value','1','Position',[35 40 95 70]);
add_block('simulink/Sources/Constant',[model '/vq'],'Value','12','Position',[35 100 95 130]);
add_block('simulink/Sources/Constant',[model '/Tload'],'Value','3','Position',[35 160 95 190]);
add_block('simulink/User-Defined Functions/MATLAB Function',[model '/dq_equations'],'Position',[290 65 465 215]);
chart = find(sfroot,'-isa','Stateflow.EMChart','Path',[model '/dq_equations']);
chart.Script = sprintf(['function dx = dq_equations(x,vd,vq,Tload)\n' ...
    '%% PMSM in rotor dq coordinates; SI inputs and states.\n' ...
    'Rs=0.12; Ld=0.0004; Lq=0.0008; flux=0.085; J=0.006; B=0.001; pairs=4;\n' ...
    'id=x(1); iq=x(2); omega=x(3); we=pairs*omega;\n' ...
    'Te=1.5*pairs*(flux+(Ld-Lq)*id)*iq;\n' ...
    'opposing=Tload*omega/sqrt(omega*omega+0.04);\n' ...
    'dx=[(vd-Rs*id+we*Lq*iq)/Ld; (vq-Rs*iq-we*(Ld*id+flux))/Lq;\n' ...
    ' (Te-opposing-B*omega)/J; omega];\n' ...
    'end\n']);
add_block('simulink/Signal Routing/Demux',[model '/split_derivatives'],'Outputs','4','Position',[530 55 535 265]);
add_block('simulink/Signal Routing/Mux',[model '/state_vector'],'Inputs','4','Position',[760 55 765 265]);
names = {'id','iq','omega_m','theta_m'};
for k=1:4
    y=45+(k-1)*65;
    add_block('simulink/Continuous/Integrator',[model '/' names{k}],'InitialCondition','0','Position',[605 y 665 y+30]);
    add_line(model,['split_derivatives/' num2str(k)],[names{k} '/1'],'autorouting','on');
    add_line(model,[names{k} '/1'],['state_vector/' num2str(k)],'autorouting','on');
end
add_line(model,'state_vector/1','dq_equations/1','autorouting','on');
add_line(model,'vd/1','dq_equations/2','autorouting','on');
add_line(model,'vq/1','dq_equations/3','autorouting','on');
add_line(model,'Tload/1','dq_equations/4','autorouting','on');
add_line(model,'dq_equations/1','split_derivatives/1','autorouting','on');
add_block('simulink/Sinks/To Workspace',[model '/state_record'],'VariableName','states', ...
    'SaveFormat','Structure With Time','Position',[830 80 940 120]);
add_line(model,'state_vector/1','state_record/1','autorouting','on');
add_block('simulink/Sinks/Scope',[model '/dq_and_motion'],'Position',[845 180 910 220]);
add_line(model,'state_vector/1','dq_and_motion/1','autorouting','on');
note=Simulink.Annotation(model,['PMSM dq plant | vd/vq [V], Tload [Nm]' newline ...
    'State: id [A], iq [A], omega_m [rad/s], theta_m [rad]' newline ...
    'Fixed-step RK4: 12.5 us. C drive controller is supplied separately.']);
note.Position=[35 325 730 395];
save_system(model,fullfile(folder,[model '.slx']));
out=sim(model);
states=out.get('states'); simulated=states.signals.values(end,:);
options=odeset('RelTol',1e-10,'AbsTol',1e-11);
[~,referenceStates]=ode45(@rhs,[0 .1],[0;0;0;0],options);
reference=referenceStates(end,:);
record=jsondecode(fileread(fullfile(rootFolder,'validation.json')));
c=record.numericCrosscheck.c(:)';
simError=abs(simulated-reference);cError=abs(c-reference);
assert(max(simError)<.001,'Simulink reference error exceeds tolerance');
assert(max(cError)<.001,'C reference error exceeds tolerance');
result=struct('matlabVersion',version,'model',model,'duration',.1,'step',.0000125, ...
    'simulink',simulated,'ode45',reference,'c',c,'simulinkError',simError,'cError',cError, ...
    'tolerance',.001,'passed',true);
fid=fopen(fullfile(rootFolder,'matlab-validation.json'),'w');
cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'%s\n',jsonencode(result,PrettyPrint=true));
fprintf('MATLAB + Simulink + C dq cross-check PASS; maximum C error %.8g\n',max(cError));
close_system(model,0);
end

function dx=rhs(~,x)
id=x(1);iq=x(2);w=x(3);we=4*w;Te=6*(.085+(.0004-.0008)*id)*iq;
dx=[(1-.12*id+we*.0008*iq)/.0004; ...
    (12-.12*iq-we*(.0004*id+.085))/.0008; ...
    (Te-3*w/sqrt(w*w+.04)-.001*w)/.006;w];
end
