//aquí pruebo si el cuadrotor apunta bien al ángulo que le indico con los valores P-I-D que he calculado en otro archivo
//AQUÍ ESTÁ EL FILTRO
//en el archivo 4-probarPIDtrayectoria tengo un ángulo deseado fijo.
//en el archivo 5-probarPIDtrayectoria tengo un ángulo progresivo
//en el 6, voy a cambiar. creo un while que para el programa cuando llega a la posición final.
//en el archivo 6 ya he conseguido que un dron vaya de x=240 a x=300 de una manera razonablemente buena. Ahora voy a hacer que uno le siga, y ver cómo se estira la catenaria entre medias.

//http://gafferongames.com/game-physics/integration-basics/

//lo que hago aquí es no imponer theta_objetivo, sino que se calcule solo (que vaya frenando y acelerando)
//http://linove.blogspot.com.es/2009/05/filter-design-using-scilab.html
//https://github.com/gibiansky/experiments/blob/master/quadcopter/matlab/simulate.m
clc; clear();close();
exec("catenaria3.sce");
exec("quadrotor.sce");
exec("descenso.sce");
exec("TensVerticales.sci");
exec("TensVerticales2.sci");
exec("GeoPatMakeBlock.sci");
exec("GeoVerMakeBlock.sci");
exec("Euler2R.sci");
exec('anim_block2.sce');
exec("quadrotor_izq.sce");
exec("quadrotor_izqmov.sce");
exec("quadrotor_dcha2.sce");
//exec("NeuronalNetwork.sce");


//load("Circuito_x");
//load("Circuito_y");
//Circuito_x=[34.38 48.11 61.84 75.58 89.31 103.05 116.78 130.52 144.25 157.99 171.72 185.46 199.19 212.93 226.66 240.40 254.13 267.87 281.60 295.34 295.34 300.40 305.47 310.53 315.60 320.66 325.73 330.79 335.86 340.92 345.99 351.05 356.12 361.18 366.25 371.31 376.38 381.44 386.51 391.58 391.58 399.43 407.28 415.13 422.99 430.84 438.69 446.54 454.40 462.25 470.10 477.95 485.81 493.66 501.51 509.36 517.22 525.07 532.92 540.78 540.78 552.13 563.48 574.83 586.18 597.53 608.88 620.24 631.59 642.94 654.29 665.64 676.99 688.35 699.70 711.05 722.40 733.75 745.10 756.46 756.46 759.63 762.80 765.97 769.14 772.31 775.48 778.66 781.83 785.00 788.17 794.51 797.69 800.86 804.03 807.20 810.37 813.54 816.72 816.72 819.60 822.49 825.37 828.26 831.14 834.03 836.91 839.80 842.68 845.57 848.45 851.34 854.22 857.11 859.99 862.88 865.76 868.65 871.54];
//Circuito_y=[165.24 168.91 172.59 176.27 179.95 183.63 187.31 190.99 194.67 198.35 202.02 205.70 209.38 213.06 216.74 220.42 224.10 227.78 231.46 235.14 235.14 233.71 232.29 230.86 229.44 228.01 226.59 225.17 223.74 222.32 220.89 219.47 218.04 216.62 215.20 213.77 212.35 210.92 209.50 208.08 208.08 210.51 212.95 215.39 217.82 220.26 222.70 225.13 227.57 230.01 232.44 234.88 237.32 239.75 242.19 244.63 247.06 249.50 251.94 254.38 254.38 248.32 242.27 236.21 230.16 224.10 218.05 212.00 205.94 199.89 193.83 187.78 181.72 175.67 169.62 163.56 157.51 151.45 145.404 139.35 139.35 140.62 141.89 143.16 144.43 145.70 146.97 148.24 149.51 150.78 152.05 153.32 154.59 155.86 157.13 158.40 159.67 160.94 162.21 163.49 163.49 167.73 171.98 176.23 180.48 184.73 188.98 193.23 197.48 201.73 205.98 210.23 214.48 218.73 222.98 227.23 231.48 235.73 239.98 244.23];
Circuito_x=[17.19	19.4	21.61	23.82	26.03	28.24	30.45	32.67	34.88	37.09	39.3	41.51	43.72	45.93	48.15	50.36	52.57	54.78	56.99	59.2	61.42	63.63	65.84	68.05	70.26	72.47	74.68	76.9	79.11	81.32	83.53	85.74	87.95	90.17	92.38	94.59	96.8	99.01	101.22	103.43	105.65	107.86	110.07	112.28	114.49	116.7	118.92	121.13	123.34	125.55	127.76	129.97	132.18	134.4	136.61	138.82	141.03	143.24	145.45	147.66	147.67	148.48	149.3	150.11	150.93	151.74	152.56	153.37	154.19	155.01	155.82	156.64	157.45	158.27	159.08	159.9	160.71	161.53	162.35	163.16	163.98	164.79	165.61	166.42	167.24	168.05	168.87	169.69	170.5	171.32	172.13	172.95	173.76	174.58	175.4	176.21	177.03	177.84	178.66	179.47	180.29	181.1	181.92	182.74	183.55	184.37	185.18	186,00	186.81	187.63	188.44	189.26	190.08	190.89	191.71	192.52	193.34	194.15	194.97	195.78	195.79	197.05	198.31	199.57	200.83	202.09	203.35	204.61	205.87	207.13	208.39	209.65	210.91	212.17	213.43	214.69	215.95	217.21	218.47	219.73	220.99	222.25	223.51	224.77	226.03	227.29	228.55	229.81	231.07	232.33	233.59	234.85	236.11	237.37	238.63	239.89	241.15	242.41	243.67	244.93	246.19	247.45	248.71	249.97	251.23	252.49	253.75	255.01	256.27	257.53	258.79	260.05	261.31	262.57	263.83	265.09	266.35	267.61	268.87	270.13	270.39	272.21	274.03	275.85	277.67	279.49	281.31	283.13	284.95	286.77	288.59	290.41	292.23	294.05	295.87	297.69	299.51	301.33	303.15	304.97	306.79	308.61	310.43	312.25	314.07	315.89	317.71	319.53	321.35	323.17	324.99	326.81	328.63	330.45	332.27	334.09	335.91	337.73	339.55	341.37	343.19	345.01	346.83	348.65	350.47	352.29	354.11	355.93	357.75	359.57	361.39	363.21	365.03	366.85	368.67	370.49	372.31	374.13	375.95	377.77	378.23	378.74	379.25	379.76	380.27	380.78	381.29	381.8	382.31	382.82	383.33	383.84	384.35	384.86	385.37	385.88	386.39	386.9	387.41	387.92	388.43	388.94	389.45	389.96	390.47	390.98	391.49	392,00	392.51	393.02	393.53	394.04	394.55	395.06	395.57	396.08	396.59	397.1	397.61	398.12	398.63	399.14	399.65	400.16	400.67	401.18	401.69	402.2	402.71	403.22	403.73	404.23	404.75	405.26	405.77	406.28	406.79	407.29	407.8	408.32	408.36	408.82	409.28	409.74	410.2	410.66	411.12	411.58	412.04	412.5	412.96	413.42	413.88	414.34	414.8	415.26	415.72	416.18	416.64	417.1	417.56	418.02	418.48	418.93	419.4	419.86	420.32	420.77	421.23	421.69	422.15	422.61	423.07	423.53	423.99	424.45	424.91	425.37	425.83	426.29	426.75	427.21	427.67	428.13	428.59	429.05	429.51	429.97	430.43	430.89	431.35	431.81	432.27	432.73	433.19	433.65	434.11	434.57	435.03	435.49];

Circuito_y=[165.24	166.42	167.6	168.78	169.96	171.14	172.32	173.5	174.68	175.86	177.04	178.22	179.4	180.58	181.76	182.94	184.12	185.3	186.48	187.66	188.84	190.01	191.2	192.39	193.58	194.77	195.96	197.15	198.34	199.53	200.72	201.91	203.1	204.29	205.48	206.67	207.86	209.05	210.24	211.43	212.62	213.81	215	216.19	217.38	218.57	219.76	220.95	222.14	223.33	224.52	225.71	226.9	228.09	229.28	230.47	231.66	232.85	234.04	235.23	235.14	234.68	234.22	233.76	233.3	232.84	232.38	231.92	231.46	231,00	230.54	230.08	229.62	229.16	228.7	228.24	227.78	227.32	226.86	226.4	225.94	225.48	225.02	224.56	224.1	223.64	223.18	222.72	222.26	221.8	221.34	220.88	220.42	219.96	219.5	219.04	218.58	218.12	217.66	217.2	216.74	216.28	215.82	215.36	214.9	214.44	213.98	213.52	213.06	212.6	212.14	211.68	211.22	210.76	210.3	209.84	209.38	208.92	208.46	208,00	208.08	208.86	209.64	210.42	211.2	211.98	212.76	213.54	214.32	215.1	215.88	216.66	217.44	218.22	219	219.78	220.56	221.34	222.12	222.9	223.68	224.46	225.24	226.02	226.8	227.58	228.36	229.14	229.92	230.7	231.48	232.26	233.04	233.82	234.6	235.38	236.16	236.94	237.72	238.5	239.28	240.06	240.84	241.62	242.4	243.18	243.96	244.74	245.52	246.3	247.08	247.86	248.64	249.42	250.2	250.98	251.76	252.54	253.32	254.1	254.38	252.43	250.48	248.53	246.58	244.63	242.68	240.73	238.78	236.83	234.88	232.93	230.98	229.03	227.08	225.13	223.18	221.23	219.28	217.33	215.38	213.43	211.48	209.53	207.58	205.63	203.68	201.73	199.78	197.83	195.88	193.93	191.98	190.03	188.08	186.13	184.18	182.23	180.28	178.33	176.38	174.43	172.48	170.53	168.58	166.63	164.68	162.73	160.78	158.83	156.88	154.93	152.98	151.03	149.08	147.13	145.18	143.23	141.28	139.33	139.35	139.75	140.15	140.55	140.95	141.35	141.75	142.15	142.55	142.95	143.35	143.75	144.15	144.55	144.95	145.35	145.75	146.15	146.55	146.95	147.35	147.75	148.15	148.55	148.95	149.35	149.75	150.15	150.55	150.95	151.35	151.75	152.15	152.55	152.95	153.35	153.75	154.15	154.55	154.95	155.35	155.75	156.15	156.55	156.95	157.35	157.75	158.15	158.55	158.95	159.35	159.75	160.15	160.55	160.95	161.35	161.75	162.15	162.55	162.95	163.49	164.85	166.21	167.57	168.93	170.29	171.65	173.01	174.37	175.73	177.09	178.45	179.81	181.17	182.53	183.89	185.25	186.61	187.97	189.33	190.69	192.05	193.41	194.77	196.13	197.49	198.85	200.21	201.57	202.93	204.29	205.65	207.01	208.36	209.71	211.06	212.41	213.76	215.11	216.46	217.81	219.16	220.51	221.86	223.21	224.56	225.91	227.26	228.61	229.96	231.31	232.66	234.01	235.36	236.71	238.06	239.41	240.76	242.11	243.46];
//Circuito_x=[17.19 19.85 22.51 25.17 27.84 30.50 33.16 35.83 38.49 41.15 43.81 46.48 49.14 51.80 54.47 57.13 59.79 62.45 65.12 67.78 70.44 73.11 75.77 78.43 81.09 83.76 86.42 89.08 91.74 94.41 97.07 99.73 102.40 105.06 107.72 110.38 113.05 115.71 118.37 121.04 123.70 126.36 129.03 131.69 134.35 137.01 139.68 142.34 145.00 147.67 147.67 148.652 149.63 150.61 151.59 152.58 153.56 154.54 155.52 156.50 157.49 158.47 159.45 160.43 161.41 162.40 163.38 164.36 165.34 166.32 167.31 168.29 169.27 170.25 171.23 172.22 173.20 174.18 175.16 176.14 177.13 178.11 179.09 180.077 181.05 182.04 183.02 184.00 184.98 185.96 186.95 187.93 188.91 189.89 190.87 191.86 192.84 193.82 194.80 195.79 195.79 197.31 198.83 200.35 201.87 203.40 204.92 206.44 207.96 209.49 211.01 212.53 214.05 215.58 217.10 218.62  221.67 223.19 224.71 226.23 227.76 229.28 230.80 232.32 233.85 235.37 236.89 238.41 239.94 241.46 242.98 244.50 246.03 255.16 256.68 258.21 259.73 261.25 262.77 264.30 265.82 267.34 268.86 270.39 270.39 272.59 274.79 276.99 279.19 281.39 283.59 285.79 287.99 290.19 292.39 294.59 296.79 299.00 301.20 303.40 305.60 307.80 310.00 312.20 314.40 316.60 318.80 321.00 323.20 325.41 327.61 329.81 332.01 334.21 336.41 338.61 340.81 343.01 345.21 347.41 349.61 351.82 354.02 356.22 358.42 360.62 362.82 365.02 367.22 369.42 371.62 373.82 376.02 378.23 378.23 378.84 379.45 380.07 380.68 381.30 381.91 382.53 383.14 383.76 384.37 384.99 385.60 386.22 386.83 387.45 388.06 388.68 389.29 389.91 390.52 391.14 391.75 392.37 392.98 393.60 394.21 394.83 395.44 396.06 396.67 397.29 397.90 398.52 399.13 399.75 400.36 400.98 401.59 402.21 402.82 403.44 404.05 404.67 405.28 405.90 406.51 407.13 407.74 408.36 408.36 408.91 409.47 410.03 410.59 411.15 411.71 412.27 412.83 413.39 413.95 414.51 415.07 415.63 416.19 416.75 417.31 417.86 418.42 418.98 419.54 420.10 420.66 421.22 421.78 422.34 422.90 423.46 424.02 424.58 425.14 425.70 426.26 426.81 427.37 427.93 428.49 429.05 429.61 430.17 430.73 431.29 431.85 432.41 432.97 433.53 434.09 434.65 435.21 435.77];

//drones seguidores
seguidor_x1=[10	10.05	31.37	33.91	36.20	38.11	39.54	40.42	40.75	40.69	40.46	40.28	40.31	40.65	41.30	42.22 43.38	44.73	46.23	47.85	49.57	51.35	53.18	55.06	56.98	58.94	60.92	62.93	64.95	66.98	69.03	71.09	73.16	75.25	77.33	79.43	81.53	83.63	85.75	87.86	90.00	92.12	94.25	96.39	98.52	100.67	102.82	104.97	107.12	109.27	111.42	113.58	115.74	117.92	120.08	122.24	124.41	126.58	128.75	130.93	130.90	131.68	132.07	132.53	133.03	133.56	134.14	134.73	135.37	136.03	136.70	146.15	149.00	149.45	149.79	150.16	150.55	150.97	151.42	151.87	152.35	152.85	153.37	153.90	154.45	155.01	155.60	156.19	156.79	157.41	158.04	158.68	159.32	159.99	160.66	161.33	162.01	162.70	163.40	164.10	164.82	165.53	166.25	166.99	167.71	168.45	169.18	169.93	-16.12	168.70	196.01	196.04	196.00	195.94	195.86	195.81	195.74	195.69	195.64	195.62	195.59	194.80	186.44	201.50	202.52	202.43	202.37	202.36	202.40	202.51	202.68	202.92	203.22	203.60	204.04	204.54	205.11	205.73	206.41	207.14	207.91	208.73	209.59	210.483	211.40	212.36	213.34	214.34	215.37	216.42	217.49	218.57	219.67	220.78	221.91	223.04	224.19	225.35	226.51	227.69	228.87	230.05	231.24	232.44	233.65	234.85	236.06 237.28	238.50	239.72	240.95	242.17	243.40	244.64	245.87	247.11	248.35	249.59	250.83	252.07	253.32	253.56	255.37	257.85	258.98	259.54	260.44	261.66	263.19	264.96	266.92	269.02	271.22	273.47	275.73	278.00	280.26	282.49	284.69	286.85	288.99	291.09	293.16	295.21	297.23	299.22	301.20	303.15	305.09	307.01	308.92	310.82	312.71	314.59	316.46	318.32	320.18	322.04	323.89	325.73	327.57	329.41	331.25	333.08	334.92	336.75	338.58	340.41	342.23	344.06	345.88	347.71	349.53	351.36	353.18	355.00	356.82	358.65	360.47	362.29	364.11	364.57	365.08	365.78	365.93	365.98	366.05	366.13	366.23	366.35	366.49	366.65	366.83	367.03	367.25	367.48	367.74	368.01	368.30	368.61	368.94	369.29	369.65	370.03	370.42	370.83	371.25	371.695	372.143	-19.90	379.61	412.70	413.19	413.75	414.31	414.87	415.42	415.97	416.51	417.05	417.58	418.11	418.62	419.139	419.64	420.13	420.62	421.09	421.56	422.01	422.45	422.88 423.29	423.68	424.08	424.456	424.81	425.15	425.47	425.76	426.05	426.33	426.11	426.34	426.13	425.80	425.41	424.97	424.50	424.00	423.49	422.99	422.49	422.01	421.56	421.14	420.75	420.40	420.09	419.82	419.58	419.39	419.24	419.13	419.06	419.01	419.01	419.05	419.10	419.17	419.28	419.43	419.59	419.77	419.97	420.19	420.42	420.68	420.94	421.22	421.52	421.82	422.14	422.47	422.80	423.15	423.50	423.86	424.23	424.61	424.99	425.38	425.77	426.17	426.57	426.97	427.38	427.80	428.21	428.63	429.06	429.48];

seguidor_y1=[160.00	166.42	167.60	167.50	167.09	166.43	165.60	164.80	164.23	164.09	164.48	165.37	166.65	168.19	169.86	171.57	173.26	174.90	176.49	178.02	179.49	180.90	182.29	183.64	184.95	186.22	187.48	188.72	189.94	191.15	192.35	193.54	194.73	195.90	197.08	198.25	199.42	200.58	201.75	202.91	204.07	205.23	206.40	207.56	208.72	209.88	211.05	212.21	213.38	214.54	215.70	216.87	218.04	219.20	220.37	221.54	222.71	223.88	225.05	226.22	226.11	225.63	225.93	226.16	226.37	226.59	226.78	226.97	227.13	227.29	-3.11	246.85	247.56	246.94	246.26	245.58	244.89	244.20	243.52	242.83	242.15	241.48	240.80	240.14	239.47	238.81	238.16	237.51	236.86	236.23	235.60	234.97	234.35	233.73	233.13	232.52	231.92	231.33	230.74	230.16	229.58	229.01	228.44	227.88	227.31	226.76	226.21	225.66	225.12	204.88	230.62	230.51	230.39	230.23	230.02	229.77	229.48	229.15	228.77	228.36	227.92	19.90	191.21	189.19	189.93	190.55	191.27	192.07	192.94	193.88	194.88	195.92	196.99	198.10	199.22	200.35	201.49	202.63	203.77	204.90	206.03	207.14	208.24	209.33	210.40	211.46	212.51	213.54	214.55	215.56	216.55	217.52	218.49	219.44	220.38	221.31	222.23	223.14	224.05	224.94	225.83	226.71	227.58	228.44	229.30	230.16	231.00	231.85	232.69	233.52	234.35	235.18	236.00	236.82	237.64	238.46	239.27	240.08	240.89	241.69	242.50	243.30	243.59	240.68	239.74	240.09	240.50	240.89	241.17	241.30	241.24	240.98	240.51	239.85	239.01	238.02	236.88	235.63	234.27	232.83	231.31	229.72	228.09	226.40	224.68	222.93	221.15	219.34	217.52	215.67	213.82	211.95	210.06	208.17	206.27	204.37	202.46	200.54	198.62	196.69	194.76	192.83	190.90	188.96	187.02	185.08	183.14	181.20	179.26	177.31	175.37	173.42	171.48	169.53	167.58	165.63	163.69	161.74	159.79	157.84	155.89	153.94	153.96	154.54	154.60	154.54	154.48	154.40	154.31	154.21	154.11	154.00	153.88	153.76	153.63	153.50	153.38	153.25	153.12	153.00	152.88	152.76	152.65	152.55	152.45	152.36	152.28	152.21	152.14	152.09	135.26	154.47	154.98	155.08	155.18	155.26	155.34	155.41	155.46	155.51	155.54	155.57	155.59	155.59	155.59	155.57	155.55	155.52	155.47	155.42	155.36	155.29	155.22	155.13	155.05	154.94	154.84	154.73	154.62	154.50	154.39	154.26	154.28	155.20	155.44	155.66	155.95	156.32	156.79	157.34	157.98	158.70	159.51	160.39	161.35	162.36	163.44	164.56	165.73	166.94	168.19	169.46	170.76	172.08	173.42	174.77	176.13	177.51	178.89	180.27	181.66	183.06	184.45	185.85	187.25	188.64	190.03	191.42	192.81	194.19	195.58	196.96	198.35	199.73	201.11	202.49	203.87	205.24	206.62	207.99	209.36	210.73	212.10	213.47	214.84	216.20	217.57	218.93	220.30	221.66	223.02	224.38];
//ahora, tengo que animar la catenaria
dt=0.1; dif_tiempo=0.1;
angulotheta_objetivo_dch=zeros(1,100);
angulotheta_objetivo_cen=zeros(1,100);
angulotheta_objetivo_izq=zeros(1,100);


//***PARAMÉTROS DINÁMICOS DE DRONES****
L=25; b = 1e-5; I = diag([0.5, 0.5, 1]); //L esta en centimetros
k=3e-5; m=0.5; //el momento de inercia está en [N·cm·s2]
Sab=240;
//PARROT
//L=17; b = 3.13e-5; I = diag([86, 86, 172]); //L esta en centimetros
//k=7.5e-5; m=0.38; 
//b=[N·cm·s2]  //k=[N·s2]


N=500; //N= número de elementos
angulo_max=0.6; //estaba 0.7
//***************************//
//DRONE DERECHA: defino parametros

phi_dcha=zeros(1,N); angulotheta_dcha=zeros(1,N); psi_dcha=zeros(1,N);
phi_vel_dcha=zeros(1,N);angulotheta_vel_dcha=zeros(1,N); psi_vel_dcha=zeros(1,N);
x_pos_dcha=zeros(1,N); y_pos_dcha=zeros(1,N); x_vel_dcha=zeros(1,N); y_vel_dcha=zeros(1,N);
x_acel_dcha=zeros(1,N); y_acel_dcha=zeros(1,N);
x_pos_dcha(1)=17; x_pos_dcha(2)=17; x_pos_dcha(3)=17;
y_pos_dcha(1)=165; y_pos_dcha(2)=165; y_pos_dcha(3)=165;
Thrust_dcha=zeros(1,N);
Thrust_dcha(1)=11;
Thrust_dcha(2)=11;
Thrust_dcha(3)=11;
z_dcha=zeros(1,N); z_vel_dcha=zeros(1,N); z_acel_dcha=zeros(1,N);


//***************************//
//DRONE CENTRAL: defino parametros
//CONDICIONES INICIALES DE INESTABILIDAD
//phi mueve en eje Y
//angulotheta mueve en eje X
phi=zeros(1,N);  angulotheta=zeros(1,N); psi=zeros(1,N);
phi_vel= zeros(1,N); angulotheta_vel= zeros(1,N); psi_vel= zeros(1,N); 
phi_acel= 0; angulotheta_acel= 0; psi_acel= 0;

x_vel2=zeros(1,N); x_pos2=zeros(1,N); x_acel2=zeros(1,N);
//x_pos2(1)=120; x_pos2(2)=120; x_pos2(3)=120;
x_pos2(1)=x_pos_dcha(1)-90; x_pos2(2)=x_pos_dcha(2)-90; x_pos2(3)=x_pos_dcha(3)-90;
y_acel2=zeros(1,N); y_vel2=zeros(1,N); y_pos=zeros(1,N);
y_pos(1)=165; y_pos(2)=165; y_pos(3)=165; 
z_pos2=zeros(1,N); z_vel2=zeros(1,N); z_acel2=zeros(1,N);
//z_pos2(1)=-64.5; z_pos2(2)=-64.5; z_pos2(3)=-64.5;
Thrust=zeros(1,N);
y_acel=zeros(1,N); y_vel=zeros(1,N);
//**********************************//



//***************************//
//DRON IZQUIERDA: defino parámetros
phi_izq=zeros(1,N); angulotheta_izq=zeros(1,N); psi_izq=zeros(1,N); 
phi_vel_izq=zeros(1,N); angulotheta_vel_izq=zeros(1,N); psi_vel_izq=zeros(1,N);  
z_vel_izq=zeros(1,N); z_pos_izq=zeros(1,N); 
z_acel_izq=zeros(1,N); phi_vel_izq=zeros(1,N); angulotheta_vel_izq=zeros(1,N); psi_vel_izq=zeros(1,N); 
x_acel_izq=zeros(1,N); x_vel_izq=zeros(1,N); x_pos_izq=zeros(1,N); x_vel_izq=zeros(1,N);
x_pos_izq(1)=x_pos_dcha(1)-180; x_pos_izq(2)=x_pos_dcha(2)-180; x_pos_izq(3)=x_pos_dcha(3)-180;
y_acel_izq=zeros(1,N); y_vel_izq=zeros(1,N); y_pos_izq=zeros(1,N);
//y_pos_izq(1)=-226.27; y_pos_izq(2)=-226.27; y_pos_izq(3)=-226.27;
//y_pos_izq(1)=-210; y_pos_izq(2)=-210; y_pos_izq(3)=-210;//modificar por aquí
y_pos_izq(1)=165; y_pos_izq(2)=165; y_pos_izq(3)=165;//modificar por aquí



//**********************************///
//4º DRONE
phi4=zeros(1,N);  angulotheta4=zeros(1,N); psi4=zeros(1,N);
phi_vel4= zeros(1,N); angulotheta_vel4= zeros(1,N); psi_vel4= zeros(1,N); 
phi_acel4= 0; angulotheta_acel4= 0; psi_acel4= 0;

x_vel4=zeros(1,N); x_pos4=zeros(1,N); x_acel4=zeros(1,N);
//x_pos2(1)=120; x_pos2(2)=120; x_pos2(3)=120;
x_pos4(1)=x_pos_dcha(1)-270; x_pos4(2)=x_pos_dcha(2)-270; x_pos4(3)=x_pos_dcha(3)-270;
y_acel4=zeros(1,N); y_vel4=zeros(1,N); y_pos4=zeros(1,N);
y_pos4(1)=165; y_pos4(2)=165; y_pos4(3)=165; 
z_pos4=zeros(1,N); z_vel4=zeros(1,N); z_acel4=zeros(1,N);
//z_pos2(1)=-64.5; z_pos2(2)=-64.5; z_pos2(3)=-64.5;
Thrust4=zeros(1,N);



//**********************************///
//5º DRONE
phi5=zeros(1,N);  angulotheta5=zeros(1,N); psi5=zeros(1,N);
phi_vel5= zeros(1,N); angulotheta_vel5= zeros(1,N); psi_vel5= zeros(1,N); 
phi_acel5= 0; angulotheta_acel5= 0; psi_acel5= 0;

x_vel5=zeros(1,N); x_pos5=zeros(1,N); x_acel5=zeros(1,N);
//x_pos2(1)=120; x_pos2(2)=120; x_pos2(3)=120;
x_pos5(1)=x_pos_dcha(1)-360; x_pos5(2)=x_pos_dcha(1)-360; x_pos5(3)=x_pos_dcha(1)-360;
y_acel5=zeros(1,N); y_vel5=zeros(1,N); y_pos5=zeros(1,N);
y_pos5(1)=165; y_pos5(2)=165; y_pos5(3)=165; //por pitagoras
z_pos5=zeros(1,N); z_vel5=zeros(1,N); z_acel5=zeros(1,N);
//z_pos2(1)=-64.5; z_pos2(2)=-64.5; z_pos2(3)=-64.5;
Thrust5=zeros(1,N);


x_vel_deseado=2; 



 //valores del controlador PID
//Kp=4.05; Kd=20.64; Ki=0; //muy bueno
//Kp=4.6521; Kd=10.1373; Ki=0; //muy bueno
//Kp=3.6796; Kd=12.008; Ki=0; //muy bueno
//Kp=64.06; Kd=58.48; Ki=0;
Kp=140.6659; Kd=41.36; Ki=0;
w=0.005;

//Kp=2.008; Kd=4.9687; Ki=0;

xB1=120; xB2=120; xB3=120; xB4=120; //DRONE 3+4
vectorX1=linspace(0,xB1,150);
vectorX2=linspace(0,xB2,150);
vectorX3=linspace(0,xB3,150); //DRONE 3+4
vectorX4=linspace(0,xB4,150); //DRONE 3+4
yB1=0; yB2=0; yB3=0;
z1=zeros(1,length(vectorX1));
z2=zeros(1,length(vectorX2));
z3=zeros(1,length(vectorX3));

matriz_y1=zeros(20,150);
matriz_y2=zeros(20,150);
matriz_y3=zeros(20,150);

        
[y1,x01,y01,c1] = catenaria3(xB1, yB1, Sab, vectorX1);
       
[y2,x02,y02,c2] = catenaria3(xB2, -yB1, Sab, vectorX2);
//vectorX2=vectorX1+120;
[y3,x03,y03,c3] = catenaria3(xB3, -yB1, Sab, vectorX3); //catenaria nueva

[y4,x04,y04,c4] = catenaria3(xB4, yB1, Sab, vectorX4); //catenaria nueva

y2=y2+y1(150);
[yB2]= descenso(Sab, xB2,w);
yB2=-64.54; //OBTENIDO CON WOLFRAMALPHA

yB1=yB2;

//yB1=-7.5;   EL CONTROLADOR PID SIRVE PARA TODAS LAS ALTURAS, Y TAMBIÉN PARA TODAS LAS MASAS
//yB2=-7.5;


NudoBaja=yB1;


    vectorX2=linspace(0, xB2, 150);
    [y1,x01,y01,c1] = catenaria3(xB1, NudoBaja, Sab, vectorX1);
           
    [y2,x02,y02,c2] = catenaria3(xB2, -NudoBaja, Sab, vectorX2);
    vectorX2=vectorX1+120;
    y2=y2+y1(150);
    
    vectorX3=vectorX2+120;
    
     //tensiones verticales de los extremos [kg]
    [TensV_A, TensV_B1, TensH_A, TensH_B1]= TensVerticales(c1,y01,yB1);
    [TensV_B2, TensV_C, TensH_B2, TensH_C]= TensVerticales(c2,y02,yB2);
    
    Thrust(1)=TensV_B1+TensV_B2;
    Thrust(2)=TensV_B1+TensV_B2;
    Thrust(3)=TensV_B1+TensV_B2;
    
    Thrust_izq(1)=TensV_A;
    Thrust_izq(2)=TensV_A;
    Thrust_izq(3)=TensV_A;
    
    Thrust4(1)=TensV_A;
    Thrust4(2)=TensV_A;
    Thrust4(3)=TensV_A;
    
    Thrust5(1)=TensV_A;
    Thrust5(2)=TensV_A;
    Thrust5(3)=TensV_A;
   


//*********************************
//********************************
//*******************************
Matriz_O=zeros(2,1);
matriz_x=zeros(2,1);

Kxd1=0.76; Kxp1=0.22; Kxd2=0.76; Kxp2=0.22; Kxd3=0.76; Kxp3=0.22; Kxd4=0.76; Kxp4=0.22; Kxd5=0.76; Kxp5=0.22;
Kyd1=0.76; Kyp1=0.22; Kyd2=0.76; Kyp2=0.22; Kyd3=0.76; Kyp3=0.22; Kyd4=0.76; Kyp4=0.22; Kyd5=0.76; Kyp5=0.22;
//Kxd=0.1; Kxp=0.1;
//Kyd=0.1; Kyp=0.1;

//matriz_controlador(1)=Kxp;
//matriz_controlador(2)=Kxd;
dist1=zeros(1,N);
dist2=zeros(1,N);
dist3=zeros(1,N);

altura_nudo_central=zeros(1,N);
valores_Kxp1=zeros(1,N); valores_Kxd1=zeros(1,N); valores_Kyp1=zeros(1,N); valores_Kyd1=zeros(1,N);
valores_Kxp2=zeros(1,N); valores_Kxd2=zeros(1,N); valores_Kyp2=zeros(1,N); valores_Kyd2=zeros(1,N);
valores_Kxp3=zeros(1,N); valores_Kxd3=zeros(1,N); valores_Kyp3=zeros(1,N); valores_Kyd3=zeros(1,N);
valores_Kxp4=zeros(1,N); valores_Kxd4=zeros(1,N); valores_Kyp4=zeros(1,N); valores_Kyd4=zeros(1,N);
valores_Kxp5=zeros(1,N); valores_Kxd5=zeros(1,N); valores_Kyp5=zeros(1,N); valores_Kyd5=zeros(1,N);

x_deseado=340;
y_deseado=60;


x_actual=x_pos_dcha(1);
y_actual=y_pos_dcha(1);
x_actual2=x_pos2(1)-90;
y_actual2=y_pos(1);
x_actual3=x_pos_izq(1)-180;
y_actual3=y_pos_izq(1);
x_actual4=x_pos_dcha(1)-270;
y_actual4=y_pos_dcha(1);
x_actual5=y_pos_dcha(1)-360;
y_actual5=y_pos_dcha(1);
y_vel_deseado=2; //por poner algo
yB3=114;

//estas variables las he creado para el viento. Tienen que estar inicializadas para la primera iteración
orient1=%pi/2; orient2=%pi/2; orient3=%pi/2; orient4=%pi/2; orient5=%pi/2;

//theta_objetivo=0.2;   
   j=2;
   beta2=%pi/2; beta3=%pi/2;
   while (x_vel_dcha(j) ~= x_vel_deseado)
   
   while (x_pos_dcha(j)~=x_deseado) 
       
       while (y_vel_dcha(j) ~= y_vel_deseado)
   
        while (y_pos_dcha(j)~=y_deseado)
       j=j+1;
       if j<100
           //Viento=2+2*sin(%pi/4*j)/m;
           Viento=0;
       else
           Viento=0;
       end
       
       //divido viento entre m para que sea una aceleración
       
       //DRON LÍDER
       //nuevo
       
        x_deseado=Circuito_x(j);
        y_deseado=Circuito_y(j);
        
       
       
       //y_deseado=x_deseado-240;         
       x_vel_deseado=(x_deseado-x_actual)/(10*dt);
       //x_acel_deseado=(x_deseado-x_actual)/(100*dt*dt);
       x_acel_deseado=(x_vel_deseado-x_vel_dcha(j-1))/(100*dt);
       
        //DIRECCIÓN Y
       y_vel_deseado=(y_deseado-y_actual)/(10*dt);
       //y_acel_deseado=(y_deseado-y_actual)/(100*dt*dt);
       y_acel_deseado=(y_vel_deseado-y_vel_dcha(j-1))/(100*dt);

       theta_objetivo(j)=0.2;
          
            //Kd1=1; Kp1=0.8; //valores muy buenos
                        
            Ux_dcha(j)=x_acel_deseado + Kxd1*(x_vel_deseado - x_vel_dcha(j-1)) + Kxp1*(x_deseado-x_pos_dcha(j-1));
//            
        // a(j)=Ux_dcha(j)*m/(Thrust_dcha(j-1)*cos(angulotheta_dcha(j-1))); así estaba antes
        a(j)=Ux_dcha(j)*m/(Thrust_dcha(j-1)*cos(phi_dcha(j-1)));
        
         //chapuza
        if (a(j))>angulo_max then
            a(j)=angulo_max;

        elseif a(j)<-angulo_max then
            a(j)=-angulo_max;
        end
        theta_objetivo(j)=asin(a(j));
        
        
        Uy_dcha(j)=y_acel_deseado + Kyd1*(y_vel_deseado - y_vel_dcha(j-1)) + Kyp1*(y_deseado-y_pos_dcha(j-1));
        
        a_(j)=Uy_dcha(j)*m/(Thrust_dcha(j-1));
        
        if a_(j)>angulo_max then
            a_(j)=angulo_max;
        elseif a_(j)<-angulo_max then
            a_(j)=-angulo_max;
        end
        phi_objetivo(j)=-asin(a_(j));
        
       
    
    //MUEVO EL DRON DCHA
    //*********************************
    //calculo tension variable TensV_A
    xB1=sqrt((x_pos_dcha(j-1)-x_pos2(j-2))^2 + (y_pos_dcha(j-1)-y_pos(j-2))^2);
 //   yB1=-64.5;
    yB1=descenso(Sab, xB1,w);
    //aquí peta, por xB1
    altura_nudo_central(j)=yB1;
    
    vectorX1=linspace(0,xB1,150);
    
    //PRIMER TRAMO: Catenaria o Spline según distancia
    if ((xB1<75)|(xB1>165))
        //Cubic Spline
        disp("Cubic spline");
    else
        [y1,x01,y01,c1] = catenaria3(xB1, -yB1, Sab, vectorX1);
        dist1(j)=xB1;
        [TensV_A, TensV_B1, TensH_A, TensH_B1]= TensVerticales(c1,y01,yB1);
    //plot(sqrt((x_pos_dcha(j)-x_pos2(j-1))^2 - (y_pos_dcha(j)-y_pos(j-1))^2),'or'); //sirve para ver la distancia real de los hilos
end

   
    
    [z_acel_dcha(j), phi_dcha(j), angulotheta_dcha(j), psi_dcha(j), phi_vel_dcha(j), angulotheta_vel_dcha(j), psi_vel_dcha(j), Thrust_dcha(j)] = quadrotor_dcha2(phi_dcha(j-1), angulotheta_dcha(j-1), psi_dcha(j-1), phi_vel_dcha(j-1),phi_vel_dcha(j-2), angulotheta_vel_dcha(j-1),angulotheta_vel_dcha(j-2), psi_vel_dcha(j-1),psi_vel_dcha(j-2), 0, TensV_A, z_vel_dcha(j-1),z_vel_dcha(j-2),z_dcha(j-1), TensH_B1, 0,theta_objetivo(j),phi_objetivo(j-1));
    
    if abs(Thrust_dcha(j))>20
        Thrust_dcha(j)=20;
    end
    

    
    x_acel_dcha(j)=(cos(psi_dcha(j))*cos(angulotheta_dcha(j)))*(TensH_B1)/m + (sin(psi_dcha(j))*sin(phi_dcha(j))+cos(psi_dcha(j))*sin(angulotheta_dcha(j))*cos(phi_dcha(j)))*Thrust_dcha(j)/m + Viento*cos(orient1);
 
    x_vel_dcha(j)=x_vel_dcha(j-1)+ x_acel_dcha(j)/2*dt;
    //x_vel_dcha(j)=x_vel_dcha(j-1)+(x_acel_dcha(j)-x_acel_dcha(j-1))*dt;
    x_pos_dcha(j)=x_pos_dcha(j-1)+ x_vel_dcha(j)/2*dt;
    x_actual=x_pos_dcha(j);
    
    //muevo en Y
    y_acel_dcha(j)=(sin(psi_dcha(j))*sin(angulotheta_dcha(j))*cos(phi_dcha(j))-cos(psi_dcha(j))*sin(phi_dcha(j)))*Thrust_dcha(j)/m - Viento*sin(orient1);
    
    y_vel_dcha(j)=y_vel_dcha(j-1)+ y_acel_dcha(j)/2*dt;
    
    y_pos_dcha(j)=y_pos_dcha(j-1)+ y_vel_dcha(j)/2*dt;
    y_actual=y_pos_dcha(j);
    
    //orientación drone
    orient1=atan((y_pos_dcha(j)-y_pos_dcha(j-1))/(x_pos_dcha(j)-x_pos_dcha(j-1)));
    
       
    
    if j>4 then
            
      P_ex= (theta_objetivo(j)-angulotheta_dcha(j))/(theta_objetivo(j))*100;
      if abs(P_ex)> 1 & abs(P_ex)<=3
          mu=0.5*P_ex-0.5;
          Kxd1=abs(Kxd1 + 0.5*mu*(theta_objetivo(j)-angulotheta_dcha(j)));
      elseif abs(P_ex)>3 & abs(P_ex)<=5
          mu=-0.5*P_ex+2.5;
          Kxd1=abs(Kxd1 + 0.5*mu*(theta_objetivo(j)-angulotheta_dcha(j)));
      elseif abs(P_ex)>4 & abs(P_ex)<=6.5
          mu=(P_ex-4)/2.5;
          Kxp1=abs(Kxp1 + 0.5*mu*(theta_objetivo(j)-angulotheta_dcha(j)));
      elseif abs(P_ex)>6.5 & abs(P_ex)<=9
          mu=(9-P_ex)/2.5;
          Kxp1=abs(Kxp1 + 0.5*mu*(theta_objetivo(j)-angulotheta_dcha(j)));        
      end
end


if j>4 then
    P_ey=(phi_objetivo(j)-phi_dcha(j))/(phi_objetivo(j))*100;

         if abs(P_ey)> 1 & abs(P_ey)<=3
          mu=0.5*P_ey-0.5;
          Kyd1=abs(Kyd1 + 0.5*mu*(phi_objetivo(j)-phi_dcha(j)));
      elseif abs(P_ey)>3 & abs(P_ey)<=5
          mu=-0.5*P_ey+2.5;
          Kyd1=abs(Kyd1 + 0.5*mu*(phi_objetivo(j)-phi_dcha(j)));
      elseif abs(P_ey)>4 & abs(P_ey)<=6.5
          mu=(P_ey-4)/2.5;
          Kyp1=abs(Kyp1 + 0.5*mu*(phi_objetivo(j)-phi_dcha(j)));
      elseif abs(P_ey)>6.5 & abs(P_ey)<=9
          mu=(9-P_ey)/2.5;
          Kyp1=abs(Kyp1 + 0.5*mu*(phi_objetivo(j)-phi_dcha(j)));        
      end
end

valores_Kxp1(j)=Kxp1;
valores_Kxd1(j)=Kxd1;
valores_Kyp1(j)=Kyp1;
valores_Kyd1(j)=Kyd1;


disp(j);

    
    //*********************************
    //LE SIGUE EL DRON CENTRAL
    
//    x_deseado2=x_pos_dcha(j-1)-120; //si pongo x_deseado=x_pos_dcha(j-1)-120 -> el angulo tetha me sale muy irregular
//    y_deseado2=y_deseado; //esto lo pongo para inicializar
//nuevo
x_deseado2=seguidor_x1(j);
y_deseado2=seguidor_y1(j);
    

    if j>4
        //calculo el ángulo que forman el drone líder y el siguiente
        //alfa1=atan((x_pos_dcha(j)-x_pos2(j-1))/(y_pos_dcha(j-1)-y_pos(j)));
        //alfa1=alfa1+3.14/2;    
        
        if (x_pos_dcha(j)-x_pos_dcha(j-1))<0
           if (y_pos_dcha(j)-y_pos_dcha(j-1))>0
               beta1=%pi - atan(abs((y_pos_dcha(j)-y_pos_dcha(j-1))/(x_pos_dcha(j)-x_pos_dcha(j-1))));
           else
               beta1=%pi + atan(abs((y_pos_dcha(j)-y_pos_dcha(j-1))/(x_pos_dcha(j)-x_pos_dcha(j-1))));
           end
       else
           if (y_pos_dcha(j)-y_pos_dcha(j-1))>0
               beta1=atan((y_pos_dcha(j)-y_pos_dcha(j-1))/(x_pos_dcha(j)-x_pos_dcha(j-1)));
           else
               beta1=atan((y_pos_dcha(j)-y_pos_dcha(j-1))/(x_pos_dcha(j)-x_pos_dcha(j-1)));
           end
       end
        //beta1=atan((y_pos_dcha(j)-y_pos_dcha(j-1))/(x_pos_dcha(j)-x_pos_dcha(j-1)));
        //alfa1=3.14-beta1+alfa1;
        alfa1=%pi-(beta1-beta2)/2;
        landaX=80*cos(alfa1);//120
        landaY=80*sin(alfa1);

        x_deseado2=x_pos_dcha(j)+landaX*cos(beta1)-landaY*sin(beta1);
        y_deseado2=y_pos_dcha(j)+landaX*sin(beta1)+landaY*sin(beta1);


    end

    x_vel_deseado2=(x_deseado2-x_actual2)/(10*dt);
    x_acel_deseado2=(x_vel_deseado2-x_vel2(j-1))/(100*dt);
    y_vel_deseado2=(y_deseado2-y_actual2)/(10*dt);
   // y_acel_deseado2=(y_deseado2-y_actual2)/(100*dt*dt);
   y_acel_deseado2=(y_vel_deseado2-y_vel(j-1))/(100*dt);
       


//theta_objetivo=0.2;   
    //CALCULO TENSIONES VARIABLES POR ESTAR MOVIÉNDOSE LOS DRONES
   xB2=sqrt((x_pos2(j-1)-x_pos_izq(j-2))^2 + (y_pos(j-1)-y_pos_izq(j-2))^2);
   //yB2=abs(yB1)-abs(yB3);
   yB2=descenso(Sab,xB2,w); //ponia 200
   vectorX2=linspace(0, xB2, 150);
   
   //SEGUNDO TRAMO: Catenaria o Spline según distancia
   if ((xB2<75)|(xB2>165))
       //Cubic Spline
       disp("segunda cubic");
   else
       //catenaria
       [y2,x02,y02,c2] = catenaria3(xB2, yB1, Sab, vectorX2); 
       dist2(j)=xB2;
       [TensV_B2, TensV_C, TensH_B2, TensH_C]= TensVerticales(c2,y02,yB2);
       end

       
       theta_objetivo2(j)=0.2;
                //Kd1=1; Kp1=0.8; //valores muy buenos
                Ux(j)=x_acel_deseado2 + Kxd2*(x_vel_deseado2 - x_vel2(j-1)) + Kxp2*(x_deseado2-x_pos2(j-1));
//         
         a2(j)=Ux(j)*m/(Thrust(j-1)*cos(angulotheta(j-1)));
         //chapuza
        if a2(j)>angulo_max then
            a2(j)=angulo_max;
        elseif a2(j)<-angulo_max then
            a2(j)=-angulo_max;
        end
        theta_objetivo2(j)=asin(a2(j));
        
        
        Uy(j)=y_acel_deseado2 + Kyd2*(y_vel_deseado2 - y_vel(j-1)) + Kyp2*(y_deseado2-y_pos(j-1));
        
        a_2(j)=Uy(j)*m/(Thrust(j-1));
        
        if a_2(j)>angulo_max then
            a_2(j)=angulo_max;
        elseif a_2(j)<-angulo_max then
            a_2(j)=-angulo_max;
        end
        phi_objetivo2(j)=-asin(a_2(j));
        
        //si los dron 1 y 2 ser acercan mucho, freno el drone 2
        if dist1(j-1)<100
            disp("entro");
            phi_objetivo2(j)=0.001;
            theta_objetivo2(j)=0.001;
        end
        
        
    
    [z_acel2(j), phi(j), angulotheta(j), psi(j), phi_vel(j), angulotheta_vel(j), psi_vel(j), Thrust(j)] = quadrotor_dcha2(phi(j-1), angulotheta(j-1), psi(j-1), phi_vel(j-1),phi_vel(j-2), angulotheta_vel(j-1),angulotheta_vel(j-2), psi_vel(j-1),psi_vel(j-2), 0, TensV_B2+TensV_B1, z_vel2(j-1),z_vel2(j-2),z_pos2(j-1), TensH_B1, TensH_B2 ,theta_objetivo2(j),phi_objetivo2(j));
    if Thrust(j)>20
        Thrust(j)=20;
    end
    

    x_acel2(j)=(cos(psi(j))*cos(angulotheta(j)))*(TensH_B1-TensH_B2)/m + (sin(psi(j))*sin(phi(j))+cos(psi(j))*sin(angulotheta(j))*cos(phi(j)))*Thrust(j)/m + Viento*cos(orient2);
 
    x_vel2(j)=x_vel2(j-1)+ x_acel2(j)/2*dt;
    //x_vel_dcha(j)=x_vel_dcha(j-1)+(x_acel_dcha(j)-x_acel_dcha(j-1))*dt;
    x_pos2(j)=x_pos2(j-1)+ x_vel2(j)/2*dt;
    x_actual2=x_pos2(j);
    
    distancia1(j)=x_pos_dcha(j)-x_pos2(j); //distancia entre el dron derecho y el central. Me sirve para calcular la tensión de la catenaria.
    
    //DIRECCIÓN Y
     y_acel(j)=(sin(psi(j))*sin(angulotheta(j))*cos(phi(j))-cos(psi(j))*sin(phi(j)))*Thrust(j)/m - Viento*sin(orient2);
    y_vel(j)=y_vel(j-1)+ y_acel(j)/2*dt;
    y_pos(j)=y_pos(j-1)+ y_vel(j)/2*dt;
    y_actual2=y_pos(j);
    
    //orientación drone
    orient2=atan((y_pos(j)-y_pos(j-1))/(x_pos2(j)-x_pos2(j-1)));
    
    
       if j>4 then
      P_ex= (theta_objetivo2(j)-angulotheta(j))/(theta_objetivo2(j))*100;
        
      if abs(P_ex)> 1 & abs(P_ex)<=3
          mu=0.5*P_ex-0.5;
          Kxd2=abs(Kxd1 + 0.5*mu*(theta_objetivo2(j)-angulotheta(j)));
      elseif abs(P_ex)>3 & abs(P_ex)<=5
          mu=-0.5*P_ex+2.5;
          Kxd2=abs(Kxd1 + 0.5*mu*(theta_objetivo2(j)-angulotheta(j)));
      elseif abs(P_ex)>4 & abs(P_ex)<=6.5
          mu=(P_ex-4)/2.5;
          Kxp2=abs(Kxp1 + 0.5*mu*(theta_objetivo2(j)-angulotheta(j)));
      elseif abs(P_ex)>6.5 & abs(P_ex)<=9
          mu=(9-P_ex)/2.5;
          Kxp2=abs(Kxp1 + 0.5*mu*(theta_objetivo2(j)-angulotheta(j)));        
      end
end

if j>4 then
    P_ey=(phi_objetivo2(j)-phi(j))/(phi_objetivo2(j))*100;
//    if abs(P_ey)>4
//         Kyp2=abs(Kyp2 + 0.5*(phi_objetivo2(j)-phi(j)));
//    elseif abs(P_ey)>1 & abs(P_ey)<=4
//         Kyd2=abs(Kyd2 + 0.5*(phi_objetivo2(j)-phi(j)));
//    end
     if abs(P_ey)> 1 & abs(P_ey)<=3
          mu=0.5*P_ey-0.5;
          Kyd2=abs(Kyd1 + 0.5*mu*(phi_objetivo2(j)-phi(j)));
      elseif abs(P_ey)>3 & abs(P_ey)<=5
          mu=-0.5*P_ey+2.5;
          Kyd2=abs(Kyd1 + 0.5*mu*(phi_objetivo2(j)-phi(j)));
      elseif abs(P_ey)>4 & abs(P_ey)<=6.5
          mu=(P_ey-4)/2.5;
          Kyp2=abs(Kyp1 + 0.5*mu*(phi_objetivo2(j)-phi(j)));
      elseif abs(P_ey)>6.5 & abs(P_ey)<=9
          mu=(9-P_ey)/2.5;
          Kyp2=abs(Kyp1 + 0.5*mu*(phi_objetivo2(j)-phi(j)));        
      end
   // disp(P_ex,P_ey);
end

valores_Kxp2(j)=Kxp2;
valores_Kxd2(j)=Kxd2;
valores_Kyp2(j)=Kyp2;
valores_Kyd2(j)=Kyd2;


    
    //LE SIGUE EL DRON IZQUIERDO
    //************************************
    
    x_deseado3=x_pos2(j);
    y_deseado3=y_pos(j);
       
    if j>4
        
        //calculo el ángulo que forman el drone líder y el siguiente
        if (x_pos2(j)-x_pos2(j-1))<0
           if (y_pos(j)-y_pos(j-1))>0
               beta2=%pi - atan(abs((y_pos(j)-y_pos(j-1))/(x_pos2(j)-x_pos2(j-1))));
           else
               beta2=%pi + atan(abs((y_pos(j)-y_pos(j-1))/(x_pos2(j)-x_pos2(j-1))));
           end
       else
           if (y_pos(j)-y_pos(j-1))>0
               beta2=atan((y_pos(j)-y_pos(j-1))/(x_pos2(j)-x_pos2(j-1)));
           else
               beta2=atan((y_pos(j)-y_pos(j-1))/(x_pos2(j)-x_pos2(j-1)));
           end
       end
       
       
        // beta2=atan(abs((y_pos(j)-y_pos(j-1))/(x_pos2(j)-x_pos2(j-1))));
        ang(j)=beta2;
        alfa1=%pi-(beta2-beta3)/2;
        landaX=80*cos(alfa1);//200
        landaY=80*sin(alfa1);//200
              
        x_deseado3=x_pos2(j)+landaX*cos(beta2)-landaY*sin(beta2);
        y_deseado3=y_pos(j)+landaX*sin(beta2)+landaY*sin(beta2);
    end


       
    x_vel_deseado3=(x_deseado3-x_actual3)/(10*dt);
    //x_acel_deseado3=(x_deseado3-x_actual3)/(100*dt*dt);
    x_acel_deseado3=(x_vel_deseado3-x_vel_izq(j-1))/(100*dt);
    y_vel_deseado3=(y_deseado3-y_actual3)/(10*dt);
    //y_acel_deseado3=(y_deseado3-y_actual3)/(100*dt*dt);
    y_acel_deseado3=(y_vel_deseado3-y_vel_izq(j-1))/(100*dt);
 
       
       theta_objetivo3(j)=0.2;
          
            //Kd1=1; Kp1=0.8; //valores muy buenos
            Ux_izq(j)=x_acel_deseado3 + Kxd3*(x_vel_deseado3 - x_vel_izq(j-1)) + Kxp3*(x_deseado3-x_pos_izq(j-1));

         a3(j)=Ux_izq(j)*m/(Thrust_izq(j-1)*cos(angulotheta(j-1)));
         //chapuza
        if a3(j)>angulo_max then
            a3(j)=angulo_max;
        elseif a3(j)<-angulo_max then
            a3(j)=-angulo_max;
        end
        theta_objetivo3(j)=asin(a3(j));
        
        
        //DIRECCIÓN Y

       
        Uy_izq(j)=y_acel_deseado3 + Kyd3*(y_vel_deseado3 - y_vel_izq(j-1)) + Kyp3*(y_deseado3-y_pos_izq(j-1));
        
        a_3(j)=Uy_izq(j)*m/(Thrust_izq(j-1));
        
        if a_3(j)>angulo_max then
            a_3(j)=angulo_max;
        elseif a_3(j)<-angulo_max then
            a_3(j)=-angulo_max;
        end
        phi_objetivo3(j)=-asin(a_3(j));
        
        //si el dron 2 y 3 se acercan mucho, freno el drone 3
        if dist2(j-1)<100
            disp("entro");
            phi_objetivo3(j)=0.01;
            theta_objetivo3(j)=0.01;
        end
    
    [z_acel_izq(j), phi_izq(j), angulotheta_izq(j), psi_izq(j), phi_vel_izq(j), angulotheta_vel_izq(j), psi_vel_izq(j), Thrust_izq(j)] = quadrotor_dcha2(phi_izq(j-1), angulotheta_izq(j-1), psi_izq(j-1), phi_vel_izq(j-1),phi_vel_izq(j-2), angulotheta_vel_izq(j-1),angulotheta_vel_izq(j-2), psi_vel_izq(j-1),psi_vel_izq(j-2), 0, TensV_B2+TensV_B1, z_vel_izq(j-1),z_vel_izq(j-2),z_pos_izq(j-1), 0, TensH_B2 ,theta_objetivo3(j),phi_objetivo3(j));
    
    if Thrust_izq(j)>20
        Thrust_izq(j)=20;
    end
    

    x_acel_izq(j)=(cos(psi_izq(j))*cos(angulotheta_izq(j)))*(-TensH_B2)/m + (sin(psi_izq(j))*sin(phi_izq(j))+cos(psi_izq(j))*sin(angulotheta_izq(j))*cos(phi_izq(j)))*Thrust_izq(j)/m + Viento*cos(orient3);
 
    x_vel_izq(j)=x_vel_izq(j-1)+ x_acel_izq(j)/2*dt;
    //x_vel_dcha(j)=x_vel_dcha(j-1)+(x_acel_dcha(j)-x_acel_dcha(j-1))*dt;
    x_pos_izq(j)=x_pos_izq(j-1)+ x_vel_izq(j)/2*dt;
    x_actual3=x_pos_izq(j);
    distancia2(j)=x_pos2(j)-x_pos_izq(j); //distancia entre el dron central y el izquierdo. Me sirve para calcular la tensión vertical de las catenarias.
    
     y_acel_izq(j)=(sin(psi_izq(j))*sin(angulotheta_izq(j))*cos(phi_izq(j))-cos(psi_izq(j))*sin(phi_izq(j)))*Thrust_izq(j)/m - Viento*sin(orient3);
    y_vel_izq(j)=y_vel_izq(j-1)+ y_acel_izq(j)/2*dt;
    y_pos_izq(j)=y_pos_izq(j-1)+ y_vel_izq(j)/2*dt;
    y_actual3=y_pos_izq(j);
    

    //orientación drone
    orient3=atan((y_pos_izq(j)-y_pos_izq(j-1))/(x_pos_izq(j)-x_pos_izq(j-1)));
    
    
if j>4 then
    
        //calculo beta3
       if (x_pos_izq(j)-x_pos_izq(j-1))<0
           if (y_pos_izq(j)-y_pos_izq(j-1))>0
               beta3=%pi - atan(abs((y_pos_izq(j)-y_pos_izq(j-1))/(x_pos_izq(j)-x_pos_izq(j-1))));
           else
               beta3=%pi + atan(abs((y_pos_izq(j)-y_pos_izq(j-1))/(x_pos_izq(j)-x_pos_izq(j-1))));
           end
       else
           if (y_pos_izq(j)-y_pos_izq(j-1))>0
               beta3=atan((y_pos_izq(j)-y_pos_izq(j-1))/(x_pos_izq(j)-x_pos_izq(j-1)));
           else
               beta3=atan((y_pos_izq(j)-y_pos_izq(j-1))/(x_pos_izq(j)-x_pos_izq(j-1)));
           end
       end

        P_ex= (theta_objetivo3(j)-angulotheta_izq(j))/(theta_objetivo3(j))*100;
      if abs(P_ex)> 1 & abs(P_ex)<=3
          mu=0.5*P_ex-0.5;
          Kxd3=abs(Kxd3 + 0.5*mu*(theta_objetivo3(j)-angulotheta_izq(j)));
      elseif abs(P_ex)>3 & abs(P_ex)<=5
          mu=-0.5*P_ex+2.5;
          Kxd3=abs(Kxd3 + 0.5*mu*(theta_objetivo3(j)-angulotheta_izq(j)));
      elseif abs(P_ex)>4 & abs(P_ex)<=6.5
          mu=(P_ex-4)/2.5;
          Kxp3=abs(Kxp3 + 0.5*mu*(theta_objetivo3(j)-angulotheta_izq(j)));
      elseif abs(P_ex)>6.5 & abs(P_ex)<=9
          mu=(9-P_ex)/2.5;
          Kxp3=abs(Kxp3 + 0.5*mu*(theta_objetivo3(j)-angulotheta_izq(j)));        
      end
end

if j>4 then
    P_ey=(phi_objetivo3(j)-phi_izq(j))/(phi_objetivo3(j))*100;
    if abs(P_ey)> 1 & abs(P_ey)<=3
          mu=0.5*P_ey-0.5;
          Kyd3=abs(Kyd3 + 0.5*mu*(phi_objetivo3(j)-phi_izq(j)));
      elseif abs(P_ey)>3 & abs(P_ey)<=5
          mu=-0.5*P_ey+2.5;
          Kyd3=abs(Kyd3 + 0.5*mu*(phi_objetivo3(j)-phi_izq(j)));
      elseif abs(P_ey)>4 & abs(P_ey)<=6.5
          mu=(P_ey-4)/2.5;
          Kyp3=abs(Kyp3 + 0.5*mu*(phi_objetivo3(j)-phi_izq(j)));
      elseif abs(P_ey)>6.5 & abs(P_ey)<=9
          mu=(9-P_ey)/2.5;
          Kyp3=abs(Kyp3 + 0.5*mu*(phi_objetivo3(j)-phi_izq(j)));        
      end
//    if abs(P_ey)>4
//         Kyp3=abs(Kyp3 + 0.5*(phi_objetivo3(j)-phi_izq(j)));
//
//    elseif abs(P_ey)>1 & abs(P_ey)<=4
//         Kyd3=abs(Kyd3 + 0.5*(phi_objetivo3(j)-phi_izq(j)));
//    end
end

valores_Kxp3(j)=Kxp3;
valores_Kxd3(j)=Kxd3;
valores_Kyp3(j)=Kyp3;
valores_Kyd3(j)=Kyd3;

        end
end
end
end
//Kp=140.6659; Kd=41.36; Ki=0; buenos
//Kp=64.06; Kd=58.48; Ki=0;
// Kp=141.86; Kd=43.36; Ki=0;
//http://www.mathworks.com/matlabcentral/fileexchange/40052-pd-control-of-quadrotor/content/PDControlOfQuadrotor/quad_control_Main.m
//http://www.sersc.org/journals/IJCA/vol6_no5/32.pdf


//FILTRO
//

//x = [zeros(130,1);
//  1.2*ones(370,1) + 0.2*cos(2*%pi*[0:369]'/90)];
//y = x + 0.1*rand(500,1,'normal');
//[h,hm,fr]=wfir("lp",33,[.05 0],"hm",[0 0]);
//z = filter(h,1,y);

