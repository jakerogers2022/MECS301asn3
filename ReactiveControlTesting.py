def reactive_control(sensor_data):
    left_dist = sensor_data[1]
    right_dist = sensor_data[2]
    front_dist = sensor_data[0]
    if right_dist < 0:
        right_dist = 999
    if left_dist < 0:
        left_dist = 999
    if front_dist < 0:
        front_dist = 999

    print(sensor_data)
    if left_dist <= 0.4 and front_dist <= 0.4 and right_dist <= 0.4:
        return 'T'
    elif left_dist <= 0.4 and front_dist <= 0.4:
        return 'R'
    elif right_dist <= 0.4 and front_dist <= 0.4:
        return 'L'
    return 'F'

def evaluate(predictions, targets):
        if len(predictions) != len(targets):
            return "predictions and targets are not the same length!"

        num_correct = 0
        for i, direction in enumerate(targets):
            if predictions[i] == targets[direction]:
                num_correct += 1

        return num_correct / len(targets)

mix_train_data = {(0.18886572122573853, -1000.0, 0.29489263892173767): 'L',
                       (0.19246011972427368, 0.27303528785705566, 0.4725402295589447): 'T',
                       (-1000.0, -1000.0, 0.1216597780585289): 'F',
                       (-1000.0, -1000.0, 0.5008593797683716): 'F',
                       (0.4662621319293976, -1000.0, 0.2998603284358978): 'F',
                       (0.11432650685310364, -1000.0, 0.49349287152290344): 'L',
                       (-1000.0, 0.16683532297611237, -1000.0): 'F',
                       (-1000.0, 0.4849688410758972, 0.1722767949104309): 'F',
                       (0.36776286363601685, 0.48489582538604736, -1000.0): 'L',
                       (0.11320717632770538, -1000.0, -1000.0): 'L',
                       (0.4656304121017456, -1000.0, -1000.0): 'F',
                       (0.36713093519210815, 0.4793507158756256, -1000.0): 'R',
                       (0.19279010593891144, 0.20319049060344696, -1000.0): 'R',
                       (0.1432543843984604, 0.20479263365268707, -1000.0): 'R',
                       (0.23664286732673645, 0.40605753660202026, -1000.0): 'R',
                       (-1000.0, -1000.0, 0.4744504690170288): 'F',
                       (-1000.0, 0.4902779757976532, -1000.0): 'F',
                       (0.16335883736610413, -1000.0, 0.4415828585624695): 'L',
                       (-1000.0, 0.17930111289024353, 0.4783061742782593): 'F',
                       (0.13843156397342682, 0.2047196328639984, -1000.0): 'R',
                       (-1000.0, -1000.0, 0.22309862077236176): 'F',
                       (-1000.0, 0.2079465538263321, 0.44593536853790283): 'F',
                       (-1000.0, 0.3476245701313019, -1000.0): 'F',
                       (0.19611884653568268, -1000.0, 0.17517587542533875): 'R',
                       (0.47354668378829956, 0.4263686537742615, -1000.0): 'F',
                       (-1000.0, 0.49951663613319397, 0.352539598941803): 'F',
                       (0.22046276926994324, 0.49880141019821167, 0.2516286075115204): 'T',
                       (-1000.0, -1000.0, 0.17484889924526215): 'F',
                       (0.24679458141326904, 0.37764638662338257, 0.373322069644928): 'T',
                       (0.1179201677441597, 0.16323581337928772, -1000.0): 'R',
                       (0.11907172203063965, -1000.0, 0.4719002842903137): 'L',
                       (-1000.0, 0.43713298439979553, 0.2100927233695984): 'F',
                       (0.4946841597557068, -1000.0, -1000.0): 'F',
                       (0.1934836506843567, 0.4065283238887787, -1000.0): 'R',
                       (0.14272016286849976, 0.1979454755783081, -1000.0): 'R',
                       (-1000.0, -1000.0, -1000.0): 'F',
                       (0.4886481761932373, -1000.0, 0.4722394645214081): 'F',
                       (0.16812939941883087, 0.45914918184280396, -1000.0): 'R',
                       (0.11304689943790436, -1000.0, 0.2653895616531372): 'L',
                       (0.24596309661865234, 0.2498926967382431, 0.5007567405700684): 'T',
                       (0.49938568472862244, 0.25020188093185425, 0.5038331151008606): 'F',
                       (0.16292355954647064, -1000.0, -1000.0): 'R',
                       (0.2389470338821411, -1000.0, 0.447404146194458): 'L',
                       (-1000.0, 0.30519160628318787, 0.3537727892398834): 'F',
                       (0.4454415738582611, -1000.0, 0.1482175886631012): 'F',
                       (0.4690057337284088, 0.48210495710372925, -1000.0): 'F',
                       (0.16980811953544617, 0.3732264041900635, 0.37402617931365967): 'T',
                       (0.11840330064296722, 0.22482730448246002, -1000.0): 'R',
                       (0.19094285368919373, -1000.0, 0.37234944105148315): 'L',
                       (0.13888894021511078, 0.4835090935230255, -1000.0): 'R',
                       (0.2938598096370697, 0.17154090106487274, -1000.0): 'R',
                       (0.45002952218055725, 0.49831652641296387, 0.24978582561016083): 'F',
                       (-1000.0, -1000.0, 0.3537442684173584): 'F',
                       (0.23689614236354828, -1000.0, -1000.0): 'R',
                       (0.16832999885082245, -1000.0, 0.22385288774967194): 'L',
                       (-1000.0, 0.17603366076946259, 0.4859517812728882): 'F',
                       (0.46752336621284485, 0.18278983235359192, -1000.0): 'F',
                       (0.26373186707496643, -1000.0, 0.46979808807373047): 'L',
                       (0.269019216299057, 0.48895263671875, -1000.0): 'R',
                       (0.11160784959793091, 0.4868141710758209, -1000.0): 'R'}

center_only_train_data = {(0.246628075838089, 0.3717547655105591, 0.3761734068393707): 'T',
                               (0.2404981255531311, -1000.0, 0.4440508186817169): 'L',
                               (-1000.0, 0.3777196407318115, -1000.0): 'F',
                               (0.24221211671829224, -1000.0, -1000.0): 'L',
                               (0.24295705556869507, -1000.0, -1000.0): 'R',
                               (0.2396658957004547, -1000.0, 0.448072224855423): 'L',
                               (-1000.0, 0.3038579225540161, 0.34866371750831604): 'F',
                               (-1000.0, 0.42180103063583374, -1000.0): 'F',
                               (-1000.0, 0.37804165482521057, -1000.0): 'F',
                               (-1000.0, 0.37913599610328674, -1000.0): 'F',
                               (0.2413637638092041, -1000.0, -1000.0): 'R',
                               (0.24542313814163208, 0.3759225010871887, 0.3722212016582489): 'T',
                               (-1000.0, 0.30607903003692627, 0.3508318364620209): 'F',
                               (-1000.0, 0.307149738073349, 0.3485839366912842): 'F',
                               (0.24101144075393677, 0.41007760167121887, -1000.0): 'R',
                               (0.24295897781848907, 0.40540051460266113, -1000.0): 'R',
                               (-1000.0, 0.42274945974349976, -1000.0): 'F',
                               (-1000.0, 0.4193663001060486, -1000.0): 'F',
                               (-1000.0, 0.37641075253486633, -1000.0): 'F',
                               (0.24674029648303986, 0.37666746973991394, 0.37345507740974426): 'T',
                               (0.2391931414604187, -1000.0, 0.4467436373233795): 'L',
                               (0.24250562489032745, 0.4049191176891327, -1000.0): 'R'}

#FOR TESTING REACTIVE CONTROL
pridicts = []
for dict in mix_train_data:
    pridicts.append(reactive_control(dict))


print(evaluate(pridicts, mix_train_data))

