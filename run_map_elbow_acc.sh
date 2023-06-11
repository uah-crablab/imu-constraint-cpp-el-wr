#!/bin/sh

# compile:
# g++ -O3 -I .\eigen-3.4.0 kinematics.cpp run_map_elbow_acc.cpp map_elbow_acc.cpp -o map_elbow_acc
# run:
# open git bash,  execute ./run_map_elbow_acc.sh 


freq=128.0 
priNoise=1.0
gyrNoise=0.005
conNoise=0.05
dofNoise=0.04
tol=0.01
lam=1e-8
max_iter=25 

sbj="S01"
file="C:\Users\howar\Documents\cpp-code2\filter-development\data\S01ImuData_uafa.txt"
rUA=(0.199227099864005, -0.00727357328427801, 0.0291780030042967)	
rUA2=(0.111186980425604, 0.0147510320541111, -0.0447595702615915)	
rFA=(0.171656430869623, -0.0245042422329814, 0.0398833178232241)	
q1_imu=(0.139547584090428, 0.785105542601398, 0.60021792796274, 0.0622430534534179)
q2_imu=(0.146049569037773, -0.98469718824436, -0.00445454676991519, -0.0949796181325648)

./map_elbow_acc ${file} ${freq} ${priNoise} ${gyrNoise} ${conNoise} ${dofNoise} -${rUA2[0]} -${rUA2[1]} -${rUA2[2]} ${rFA[0]} ${rFA[1]} ${rFA[2]} ${q1_imu[0]} ${q1_imu[1]} ${q1_imu[2]} ${q1_imu[3]} ${q2_imu[0]} ${q2_imu[1]} ${q2_imu[2]} ${q2_imu[3]} ${tol} ${lam} ${max_iter}
cp ./out.txt ../data/${sbj}"_map_elbow_acc.txt"


sbj="S02"
file="C:\Users\howar\Documents\cpp-code2\filter-development\data\S02ImuData_uafa.txt"
rUA=(0.208312913530953, -0.0874564784782658, 0.0131943089417902)	
rUA2=(0.134691866451065, -0.0142278072389192, -0.0599284261918682)	
rFA=(0.186407081352088, -0.0202874648658111, 0.0344271858950015)	
q1_imu=(0.896581010724603, -0.410304480976678, -0.0283314039467742, 0.164286504768084)
q2_imu=(0.949289315163879, 0.307427949939559, -0.0658424070666148, -0.00162146332916842)

./map_elbow_acc ${file} ${freq} ${priNoise} ${gyrNoise} ${conNoise} ${dofNoise} -${rUA2[0]} -${rUA2[1]} -${rUA2[2]} ${rFA[0]} ${rFA[1]} ${rFA[2]} ${q1_imu[0]} ${q1_imu[1]} ${q1_imu[2]} ${q1_imu[3]} ${q2_imu[0]} ${q2_imu[1]} ${q2_imu[2]} ${q2_imu[3]} ${tol} ${lam} ${max_iter}
cp ./out.txt ../data/${sbj}"_map_elbow_acc.txt"

sbj="S03"
file="C:\Users\howar\Documents\cpp-code2\filter-development\data\S03ImuData_uafa.txt"
rUA=(0.163995295087756, -0.0371030254662712, 0.0467871579524258)	
rUA2=(0.113912067001882, 0.00897749830018545, -0.0359387933677552)	
rFA=(0.157228398646173, -0.0127284720599495, 0.0379461545166430)	
q1_imu=(0.920588327671379, -0.276723075541362, -0.214744617516611, -0.172702691541818)
q2_imu=(-0.912866685848975, -0.264620388178315, 0.0249186989382453, 0.309886305716615)

./map_elbow_acc ${file} ${freq} ${priNoise} ${gyrNoise} ${conNoise} ${dofNoise} -${rUA2[0]} -${rUA2[1]} -${rUA2[2]} ${rFA[0]} ${rFA[1]} ${rFA[2]} ${q1_imu[0]} ${q1_imu[1]} ${q1_imu[2]} ${q1_imu[3]} ${q2_imu[0]} ${q2_imu[1]} ${q2_imu[2]} ${q2_imu[3]} ${tol} ${lam} ${max_iter}
cp ./out.txt ../data/${sbj}"_map_elbow_acc.txt"

sbj="S04"
file="C:\Users\howar\Documents\cpp-code2\filter-development\data\S04ImuData_uafa.txt"
rUA=(0.0970977502243495, 0.00783815514301183, 0.0253049029811593)	
rUA2=(0.0689113549356007, 0.000131911587879446, -0.0482771281542717)	
rFA=(0.170084266368930, 0.00423186728734015, 0.0304797622186110)	
q1_imu=(0.0902413480126472, 0.805901979342123, 0.57844156102057, 0.0882261825389424)
q2_imu=(0.172597942181802, -0.979962904204234, -0.0067507890701665, -0.0991820729020164)

./map_elbow_acc ${file} ${freq} ${priNoise} ${gyrNoise} ${conNoise} ${dofNoise} -${rUA2[0]} -${rUA2[1]} -${rUA2[2]} ${rFA[0]} ${rFA[1]} ${rFA[2]} ${q1_imu[0]} ${q1_imu[1]} ${q1_imu[2]} ${q1_imu[3]} ${q2_imu[0]} ${q2_imu[1]} ${q2_imu[2]} ${q2_imu[3]} ${tol} ${lam} ${max_iter}
cp ./out.txt ../data/${sbj}"_map_elbow_acc.txt"

sbj="S05"
file="C:\Users\howar\Documents\cpp-code2\filter-development\data\S05ImuData_uafa.txt"
rUA=(0.115338365032155, -0.0249594132650708, 0.0319501930585203)	
rUA2=(0.0860535405709949, 0.0109130109587779, -0.0223400952940981)	
rFA=(0.151693384047773, -0.00702978342146271, 0.0385041468808199)	
q1_imu=(0.234948175185088, -0.0243556276740833, 0.919249248241807, -0.314939641812001)
q2_imu=(0.155287385162358, 0.572947070415424, 0.804712502041962, 0.00743448513249489)

./map_elbow_acc ${file} ${freq} ${priNoise} ${gyrNoise} ${conNoise} ${dofNoise} -${rUA2[0]} -${rUA2[1]} -${rUA2[2]} ${rFA[0]} ${rFA[1]} ${rFA[2]} ${q1_imu[0]} ${q1_imu[1]} ${q1_imu[2]} ${q1_imu[3]} ${q2_imu[0]} ${q2_imu[1]} ${q2_imu[2]} ${q2_imu[3]} ${tol} ${lam} ${max_iter}
cp ./out.txt ../data/${sbj}"_map_elbow_acc.txt"

sbj="S06"
file="C:\Users\howar\Documents\cpp-code2\filter-development\data\S06ImuData_uafa.txt"
rUA=(0.107991046286083, -0.0354632764578129, 0.0174506938057439)	
rUA2=(0.0916314601749135, -0.00391877688507297, -0.0495272699064785)	
rFA=(0.132759961058356, -0.00320988039746716, 0.0297363406584653)	
q1_imu=(0.107407702910464, 0.664084394507851, 0.739444950192478, 0.0260128422415377)
q2_imu=(0.155287385162358, 0.572947070415424, 0.804712502041962, 0.00743448513249489)

./map_elbow_acc ${file} ${freq} ${priNoise} ${gyrNoise} ${conNoise} ${dofNoise} -${rUA2[0]} -${rUA2[1]} -${rUA2[2]} ${rFA[0]} ${rFA[1]} ${rFA[2]} ${q1_imu[0]} ${q1_imu[1]} ${q1_imu[2]} ${q1_imu[3]} ${q2_imu[0]} ${q2_imu[1]} ${q2_imu[2]} ${q2_imu[3]} ${tol} ${lam} ${max_iter}
cp ./out.txt ../data/${sbj}"_map_elbow_acc.txt"

sbj="S07"
file="C:\Users\howar\Documents\cpp-code2\filter-development\data\S07ImuData_uafa.txt"
rUA=(0.165306443071482, -0.0358800281999658, 0.0379846061384715)	
rUA2=(0.128794896685371, -0.0109572265167738, -0.0473900729850343)	
rFA=(0.160264252611533, -0.0130398909751767, 0.0274667855662934)	
q1_imu=(0.244854351565147, 0.835645453977793, 0.488538432711528, -0.0554366442853787)
q2_imu=(0.226272119012941, -0.957240758430191, 0.180212872071993, 0.0037920043446872)

./map_elbow_acc ${file} ${freq} ${priNoise} ${gyrNoise} ${conNoise} ${dofNoise} -${rUA2[0]} -${rUA2[1]} -${rUA2[2]} ${rFA[0]} ${rFA[1]} ${rFA[2]} ${q1_imu[0]} ${q1_imu[1]} ${q1_imu[2]} ${q1_imu[3]} ${q2_imu[0]} ${q2_imu[1]} ${q2_imu[2]} ${q2_imu[3]} ${tol} ${lam} ${max_iter}
cp ./out.txt ../data/${sbj}"_map_elbow_acc.txt"

sbj="S08"
file="C:\Users\howar\Documents\cpp-code2\filter-development\data\S08ImuData_uafa.txt"
rUA=(0.101370852001234, -0.00991433813286422, 0.0210656940898063)	
rUA2=(0.0878960522262133, 0.00881206213820066, -0.0363183259400607)	
rFA=(0.118843891576306, 0.0118759295346774, 0.0310815974860827)	
q1_imu=(0.19005085611013, 0.946634666815216, -0.17223084356564, 0.195192254457292)
q2_imu=(0.195285557512922, -0.602834371367288, 0.773595920412772, -0.00190358761060354)

./map_elbow_acc ${file} ${freq} ${priNoise} ${gyrNoise} ${conNoise} ${dofNoise} -${rUA2[0]} -${rUA2[1]} -${rUA2[2]} ${rFA[0]} ${rFA[1]} ${rFA[2]} ${q1_imu[0]} ${q1_imu[1]} ${q1_imu[2]} ${q1_imu[3]} ${q2_imu[0]} ${q2_imu[1]} ${q2_imu[2]} ${q2_imu[3]} ${tol} ${lam} ${max_iter}
cp ./out.txt ../data/${sbj}"_map_elbow_acc.txt"

sbj="S09"
file="C:\Users\howar\Documents\cpp-code2\filter-development\data\S09ImuData_uafa.txt"
rUA=(0.148772036635422, -0.0175245306377171, 0.0435550072563968)	
rUA2=(0.102177516797653, 0.0261289938770846, -0.0233916489295006)	
rFA=(0.152815865635566, 0.00612367441179352, 0.0270672442279494)	
q1_imu=(0.974957238469673, 0.060641283123323, -0.212779900222633, 0.0224884858981724)
q2_imu=(0.9109605947091, 0.404731219348962, -0.079622211734545, -0.00193348649950298)

./map_elbow_acc ${file} ${freq} ${priNoise} ${gyrNoise} ${conNoise} ${dofNoise} -${rUA2[0]} -${rUA2[1]} -${rUA2[2]} ${rFA[0]} ${rFA[1]} ${rFA[2]} ${q1_imu[0]} ${q1_imu[1]} ${q1_imu[2]} ${q1_imu[3]} ${q2_imu[0]} ${q2_imu[1]} ${q2_imu[2]} ${q2_imu[3]} ${tol} ${lam} ${max_iter}
cp ./out.txt ../data/${sbj}"_map_elbow_acc.txt"

sbj="S10"
file="C:\Users\howar\Documents\cpp-code2\filter-development\data\S10ImuData_uafa.txt"
rUA=(0.174233741692063, -0.0462583913167645, 0.0405850028507381)	
rUA2=(0.0280610678833967, 0.0305104656165486, -0.00488477840079321)	
rFA=(0.194456944387463, -0.0407280969619438, 0.0153555047214004)	
q1_imu=(0.946762235097093, -0.190044000039387, -0.180225734030188, 0.187198378829163)
q2_imu=(0.982812611166447, 0.048326043601033, -0.178135791979994, -0.00340653162627958)

./map_elbow_acc ${file} ${freq} ${priNoise} ${gyrNoise} ${conNoise} ${dofNoise} -${rUA2[0]} -${rUA2[1]} -${rUA2[2]} ${rFA[0]} ${rFA[1]} ${rFA[2]} ${q1_imu[0]} ${q1_imu[1]} ${q1_imu[2]} ${q1_imu[3]} ${q2_imu[0]} ${q2_imu[1]} ${q2_imu[2]} ${q2_imu[3]} ${tol} ${lam} ${max_iter}
cp ./out.txt ../data/${sbj}"_map_elbow_acc.txt"

sbj="S11"
file="C:\Users\howar\Documents\cpp-code2\filter-development\data\S11ImuData_uafa.txt"
rUA=(0.183109699922456, -0.0652324595995309, 0.0344161142157742)	
rUA2=(0.0801487249414143, 0.0139477411060307, -0.0368487418125299)	
rFA=(0.130382946390738, 0.00279986701881042, 0.0614796477208826)	
q1_imu=(0.197840305775973, 0.96660763532364, 0.066828285787488, 0.148535763246484)
q2_imu=(0.149588398604214, -0.728699084182648, 0.668252631879336, 0.00770556297341789)

./map_elbow_acc ${file} ${freq} ${priNoise} ${gyrNoise} ${conNoise} ${dofNoise} -${rUA2[0]} -${rUA2[1]} -${rUA2[2]} ${rFA[0]} ${rFA[1]} ${rFA[2]} ${q1_imu[0]} ${q1_imu[1]} ${q1_imu[2]} ${q1_imu[3]} ${q2_imu[0]} ${q2_imu[1]} ${q2_imu[2]} ${q2_imu[3]} ${tol} ${lam} ${max_iter}
cp ./out.txt ../data/${sbj}"_map_elbow_acc.txt"

sbj="S12"
file="C:\Users\howar\Documents\cpp-code2\filter-development\data\S12ImuData_uafa.txt"
rUA=(0.154936181805130, -0.0548231009767910, 0.0247552308717461)	
rUA2=(0.0713908370209821, 0.0170306791509705, -0.0363465098615737)	
rFA=(0.147975115493467, 0.00859971833674370, 0.0633658248904836)	
q1_imu=(0.967979556003096, -0.178595878060749, -0.0901458851391193, 0.151633805242248)
q2_imu=(0.965367305212837, 0.225360824866085, -0.131447387771735, 0.000221110311382985)

./map_elbow_acc ${file} ${freq} ${priNoise} ${gyrNoise} ${conNoise} ${dofNoise} -${rUA2[0]} -${rUA2[1]} -${rUA2[2]} ${rFA[0]} ${rFA[1]} ${rFA[2]} ${q1_imu[0]} ${q1_imu[1]} ${q1_imu[2]} ${q1_imu[3]} ${q2_imu[0]} ${q2_imu[1]} ${q2_imu[2]} ${q2_imu[3]} ${tol} ${lam} ${max_iter}
cp ./out.txt ../data/${sbj}"_map_elbow_acc.txt"

sbj="S13"
file="C:\Users\howar\Documents\cpp-code2\filter-development\data\S13ImuData_uafa.txt"
rUA=(0.187643564860542, -0.0496825044253424, 0.0432249075687725)	
rUA2=(0.0872109796220106, 0.0250335825665963, -0.0227211392525579)	
rFA=(0.146728552903295, -0.0171189208180287, 0.0334589832855594)	
q1_imu=(0.89312603746783, -0.420563808718544, -0.088366155053956, 0.132828410493177)
q2_imu=(0.993299425636443, 0.0180999919287054, 0.00739820072220356, 0.113903063824317)

./map_elbow_acc ${file} ${freq} ${priNoise} ${gyrNoise} ${conNoise} ${dofNoise} -${rUA2[0]} -${rUA2[1]} -${rUA2[2]} ${rFA[0]} ${rFA[1]} ${rFA[2]} ${q1_imu[0]} ${q1_imu[1]} ${q1_imu[2]} ${q1_imu[3]} ${q2_imu[0]} ${q2_imu[1]} ${q2_imu[2]} ${q2_imu[3]} ${tol} ${lam} ${max_iter}
cp ./out.txt ../data/${sbj}"_map_elbow_acc.txt"