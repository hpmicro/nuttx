#!groovy

def testReport = "TestReport.xml"
def cases = []
def commitid = ""
def cronSettings = "0 23 * * *"

if(BRANCH_NAME == "master"){
	cronSettings = "0 23 * * *"
} else {
    return
}


class Globals {
    static winBatBatch = [:] // buildnode: BatbBatchNum
    static linuxReportStash = [:] //buildnode: stashreport
    static String buildCodeClone = "git clone git@192.168.11.211:swtesting/rtt_build.git -b release_1.0.0"
    static String hpmSdkCloneLink = "git@192.168.11.211:oss/nuttx.git"
    static String winCloneDstDir = "D:\\hpm_sdk"
    static String exBoardList = "[]"
    static String toolchainList = "['gnu_gcc']"
    static String exBuildTypeList = "[]"
}

pipeline {
    agent {
        label "linux_node"
    }    // agent

    options {
        skipDefaultCheckout()
        buildDiscarder(logRotator(numToKeepStr: '15', artifactNumToKeepStr: '15'))
        timestamps()
    }

    triggers {
        cron(cronSettings)
    }

    stages {
        stage("Checkout repo"){
            when {
                branch BRANCH_NAME
            }
            steps {
                deleteDir()
                checkout scm
                script {
                    commitid = sh(returnStdout: true, script: 'git rev-parse --short HEAD').trim()

                }
            }
        }

        stage("Collect case list"){
            when {
                branch BRANCH_NAME
            }

            steps {
                script {
                    sh("$Globals.buildCodeClone && /home/builder/.local/bin/pytest --collect-only --hpm_sdk_dir . ci_build/test_build.py --build_base_dir build --ex_board_list \"$Globals.exBoardList\" --toolchain_list \"$Globals.toolchainList\" --ex_build_type_list \"$Globals.exBuildTypeList\"")
                    if(fileExists('caselist.csv')) {
                        echo 'caselist.csv found'
                        readFile("caselist.csv").split('\n').each { line, count ->
                            cases.add(line.split(',')[1])

                        }
                        cases =  cases[1..-1]  // ignore header column
                    }
                }
            }
        }

        stage("parallel_build"){
            when {
                branch BRANCH_NAME
            }
            steps{
                script {
                    def projectName = getProjectnName()
                    def buildStages = [:]
                    if(projectName.indexOf("linux") != -1){
                        buildStages = getParallelStageByCaseBalance(cases, "linux")
                    } else {
                        buildStages = getParallelStageByCaseBalance(cases, "windows")
                    }
                    parallel buildStages
                }
            }
        }


        stage("merge report"){

            when {
                branch BRANCH_NAME
            }

            steps {
                script {
                    def projectName = getProjectnName()
                    def buildNodes = []


                    // unstash report for each build
                    def merge_cmd = "/home/builder/.local/bin/junitparser merge"

                    if(projectName.indexOf("linux") != -1){
                        Globals.linuxReportStash.each { node, report ->
                            unstash name: report
                            merge_cmd = merge_cmd + " ci_build/" + report +".xml"
                        }
                    } else {
                        Globals.winBatBatch.each { node, num ->
                            for(int i=1; i < num + 1; i++){
                                unstash name: "report_" + node + "_" + Integer.toString(i)
                                merge_cmd = merge_cmd + " ci_build/report_" + node + "_" +  Integer.toString(i) + ".xml"
                            }
                        }
                    }

                    merge_cmd = merge_cmd + " " + testReport
                    sh(merge_cmd)
                    junit testReport
                }
                archiveArtifacts artifacts: testReport, fingerprint: true
            }
        }
    }

    post {
        always {
            echo "Done with the Build"
            script {
                def changeSets = getChangeSets().toString()
                gen_report_cmd = "python3 ci_build/generate_report.py --xml_reportfile $testReport --build_url ${BUILD_URL} --build_date \"${BUILD_TIMESTAMP}\" --build_duration \"${currentBuild.durationString}\" --git_commit $commitid --branch ${BRANCH_NAME} --cause \"${currentBuild.getBuildCauses()[0].shortDescription}\" --changesets \"${changeSets}\""
                sh(gen_report_cmd)
            }
            //archiveArtifacts artifacts: "*.zip", fingerprint: true
            //cleanWs()
        }// always
        success {
            script {
                testResult = "success"
                EmailNotification(commitid, testResult)
            }// script
        }// success
        failure {
            script {
                testResult = "failure"
                EmailNotification(commitid, testResult)
            }// script
        }// failure
        unstable {
            script {
                testResult = "unstable"
                EmailNotification(commitid, testResult)
            }// script
        }// unstable
    }// post

}// pipeline


@NonCPS
List <String> getOnlineNodeNames(filterLabel) {
    List <String> nodeNames = []
    def allNodes = Jenkins.getInstance().getNodes()
    for (int i =0; i < allNodes.size(); i++) {
        Slave node = allNodes[i]

        if (node.getComputer().isOnline() && node.getLabelString().matches(filterLabel + "(.*)")){
            nodeNames.add(node.getLabelString())
        }
  }
  return nodeNames
}

@NonCPS
def getChangeSets(){
    def changeSets = []

    def changeLogSets = currentBuild.changeSets
    for (int i = 0; i < changeLogSets.size(); i++) {
        def changeStr = ""
        def entries = changeLogSets[i].items
        for (int j = 0; j < entries.length; j++) {
            def entry = entries[j]
            changeStr = "'${entry.commitId[0..6]} revision by ${entry.author}: ${entry.msg}'"
            changeSets.add(changeStr)
        }
    }
    return changeSets
}

String getBranchName(){
    def branchName = "${BRANCH_NAME}"
    return branchName
}

String getProjectnName(){
    def projectName = "${JOB_NAME}"
    projectName = projectName.split("/")[0]
    return projectName
}

def getParallelStageByCaseBalance(cases, os){
    def buildId = "${BUILD_ID}"
    def buildNodes = []
    def branchName =  getBranchName()

    if(os == "linux"){
        buildNodes = getOnlineNodeNames("Linux_build")
    } else {
        buildNodes = getOnlineNodeNames("Windows_build")
    }

    def nodeCaseMap = [:]
    def nodeIndex = 0

    // Init nodeCaseMap
    for(node in buildNodes){
        nodeCaseMap[node] = []
    }

    //for each node add case
    for(caseName in cases){
        if(nodeIndex == buildNodes.size()){
            nodeIndex = 0
        }
        nodeCaseMap[buildNodes[nodeIndex]].add(caseName)
        nodeIndex = nodeIndex + 1
    }

    //Declear buildStage
    def buildStages = [:]
    nodeCaseMap.each { buildNode, runCases ->
        buildStages[buildNode + "_build"] = {
            node(buildNode){
                stage(buildNode + "_build"){
                    deleteDir()
                    if(os == "linux"){
                        String casesStr = ""
                        for(String caseName in runCases){
                            casesStr = casesStr + " " + caseName[0..-1]
                        }
                        if(runCases.size() != 0){
                            sh("git clone $Globals.hpmSdkCloneLink -b $branchName")
                            sh(Globals.buildCodeClone)
                            sh("export PATH=$PATH:/usr/share/segger_embedded_studio_for_risc/bin && cd ci_build && /home/builder/.local/bin/pytest --hpm_sdk_dir ${WORKSPACE}/hpm_sdk $casesStr --build_base_dir /home/builder/build/$branchName/$buildId --gnu_toolchain_dir /home/builder/riscv32-unknown-elf-newlib-multilib --andes_toolchain_dir /home/builder/nds32le-elf-newlib-v5d -n 8 --suppress-tests-failed-exit-code --junit-xml=report_${buildNode}.xml --branch_name $branchName --build_id $buildId --ex_board_list \"$Globals.exBoardList\" --toolchain_list \"$Globals.toolchainList\" --ex_build_type_list \"$Globals.exBuildTypeList\"")
                            stash includes: "ci_build/*.xml", name:"report_$buildNode"
                            Globals.linuxReportStash.put(buildNode, "report_$buildNode")
                        } else {
                            println "No case assigned"
                        }
                    } else {
                        bat("if exist $Globals.winCloneDstDir rd /q /s $Globals.winCloneDstDir")
                        bat("git config --global core.longpaths true")
                        bat("git clone $Globals.hpmSdkCloneLink -b $branchName $Globals.winCloneDstDir")
                        bat(Globals.buildCodeClone)
			            def index = 0
                        String casesStr = ""
                        if(runCases.size() != 0) {
                            // This solution is fix windonws command line length limitation
                            for(String caseName in runCases){
                                casesStr = casesStr + " " + caseName[0..-1]
                                if(casesStr.length() > 7000){ // windodws command line max length is 8191
                                    index = index + 1
                                    bat("cd ci_build && pytest --hpm_sdk_dir $Globals.winCloneDstDir $casesStr --build_base_dir D:\\build\\$branchName\\$buildId --gnu_toolchain_dir D:\\toolchain\\rv32imac-ilp32-multilib-win --andes_toolchain_dir D:\\toolchain\\nds32le-elf-newlib-v5d -n 8 --suppress-tests-failed-exit-code --junit-xml=report_${buildNode}_${index}.xml --branch_name $branchName --build_id $buildId --ex_board_list \"$Globals.exBoardList\" --toolchain_list \"$Globals.toolchainList\" --ex_build_type_list \"$Globals.exBuildTypeList\"")
                                    stash includes: "ci_build/report_${buildNode}_${index}.xml", name:"report_${buildNode}_${index}"
                                    casesStr = ""
                                }
                            }
                            if(casesStr.length() > 0){
                                index = index + 1
                                bat("cd ci_build && pytest --hpm_sdk_dir $Globals.winCloneDstDir $casesStr --build_base_dir D:\\build\\$branchName\\$buildId --gnu_toolchain_dir D:\\toolchain\\rv32imac-ilp32-multilib-win --andes_toolchain_dir D:\\toolchain\\nds32le-elf-newlib-v5d -n 8 --suppress-tests-failed-exit-code --junit-xml=report_${buildNode}_${index}.xml --branch_name $branchName --build_id $buildId --ex_board_list \"$Globals.exBoardList\" --toolchain_list \"$Globals.toolchainList\" --ex_build_type_list \"$Globals.exBuildTypeList\"")
                                stash includes: "ci_build/report_${buildNode}_${index}.xml", name:"report_${buildNode}_${index}"
                            }
                            Globals.winBatBatch.put(buildNode, index)
                        } else {
                            println "No case assigned"
                        }

                    }
                }
            }
        }
    }

    return buildStages
}


def EmailNotification(commitid, buildResult){
    def os = ""
    def project = scm.getUserRemoteConfigs()[0].getUrl().tokenize('/').last().split("\\.")[0]
    println "${currentBuild.getBuildCauses()[0].shortDescription}"
    def jenkinsBuildType = "Daily Build"

    if(currentBuild.getBuildCauses()[0].shortDescription == 'Started by timer'){
        jenkinsBuildType = "Daily Build"
    }

    if(getProjectnName().indexOf("linux") != -1){
        os = "Linux"
    } else {
        os = "Windows"
    }

    def mailList = ""
    def jobName = currentBuild.fullDisplayName
    def testResult = "Passed"
    if(buildResult == "success"){
        mailList = "sw@hpmicro.com"
        testResult = "Passed"
    } else if(buildResult == "failure"){
        mailList = "swtest@hpmicro.com"
        testResult = "Failed"
    } else {
        mailList = "sw@hpmicro.com"
        testResult = "Failed"
    }


    emailext body: '''${FILE, path="report.html"}''',
    	mimeType: 'text/html',
        subject: "[${jenkinsBuildType}]${project}-${env.BRANCH_NAME}-${commitid}-${os}-${testResult}",
        to: "${mailList}",
        replyTo: "${mailList}",
        recipientProviders: [[$class: 'CulpritsRecipientProvider']]

}// EmailNotification