#!groovytestReport

def testReport = "TestReport.xml"
def cases = []
def commitid = ""
def cronSettings = "0 0 0 * *"
def rttWorkspace = ""

if(BRANCH_NAME == "nuttx_with_hpmsdk"){
	cronSettings = "0 12 * * *"    // default set 12 clock to execute every day
} else {
    cronSettings = "0 0 31 2 *"
}


class Globals {
    static linuxReportStash = [:] //buildnode: stashreport
    static String buildCodeClone = "git clone git@192.168.11.211:swtesting/rtt_build.git -b nuttx_release_1.0.0"
    static String nuttxCodeClone = "git@192.168.11.211:oss/nuttx.git"
    // static String appsCodeClone = "git clone https://github.com/apache/nuttx-apps.git apps --depth=1 -b releases/12.0"
    static String appsPackage = "nuttx-apps-releases-12.0"
}

pipeline {
    agent {
        label "linux_node"
    }

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
                println "BRANCH_NAME：$BRANCH_NAME "
                println WORKSPACE
                deleteDir()
                // checkout scm
                script {
                    sh("$Globals.buildCodeClone && git clone $Globals.nuttxCodeClone -b $BRANCH_NAME")
                    commitid = sh(returnStdout: true, script: "cd nuttx && git rev-parse --short HEAD").trim()
                }
            }
        }

        stage("collect case list"){
            when {
                branch BRANCH_NAME
            }
            steps {
                script {
                    sh("export PATH=$PATH:/home/builder/nuttx_toolchain/riscv32-unknown-elf-newlib-multilib/bin && cd rtt_build && /home/builder/.local/bin/pytest --collect-only --project nuttx --project_src_dir ${WORKSPACE}/nuttx --build_dir ${WORKSPACE}/output test_build/test_nuttx_build/test_build.py")
                    def caseFile = "$WORKSPACE/rtt_build/caselist.csv"
                    if(fileExists(caseFile)) {
                        echo 'caselist.csv found'
                        readFile(caseFile).split('\n').each { line, count ->
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
                    def projectName = getProjectName()
                    def buildStages = [:]
                    if(projectName.indexOf("nuttx") != -1){
                        buildStages = getParallelStageByCaseBalance(cases, "linux")
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
                    rttWorkspace = "$WORKSPACE/rtt_build"
                    // unstash report for each build
                    def merge_cmd = "/home/builder/.local/bin/junitparser merge"
                    Globals.linuxReportStash.each{node, report ->
                        unstash name: report
                        merge_cmd = merge_cmd + " " + "rtt_build/${report}.xml"
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
                gen_report_cmd = "python3 rtt_build/nuttx_generate_report.py --xml_reportfile $testReport --build_url ${BUILD_URL} --build_date \"${BUILD_TIMESTAMP}\" --build_duration \"${currentBuild.durationString}\" --git_commit $commitid --branch ${BRANCH_NAME} --cause \"${currentBuild.getBuildCauses()[0].shortDescription}\" --changesets \"${changeSets}\""
                sh(gen_report_cmd)
            }
            //archiveArtifacts artifacts: "*.zip", fingerprint: true
            //cleanWs()
        }// always
        success {
            script {
                println "****Success****"
                testResult = "success"
                EmailNotification(commitid, testResult)
            }// script
        }// success
        failure {
            script {
                println "****Failure****"
                testResult = "failure"
                EmailNotification(commitid, testResult)
            }// script
        }// failure
        unstable {
            script {
                println "****Unstable****"
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

String getProjectName(){
    def projectName = "${JOB_NAME}"
    println "Job Name：${projectName}"
    projectName = projectName.split("/")[0]
    return projectName
}

def getParallelStageByCaseBalance(cases, os){
    def buildNodes = []
    def branchName = getBranchName()
    def projectName = getProjectName()

    if(os == "linux"){
        buildNodes = getOnlineNodeNames("Linux_build")
        // buildNodes = getOnlineNodeNames("qa_builder")
        println "buildNodes: ${buildNodes}"
    } else {
        buildNodes = getOnlineNodeNames("Windows_build")
    }

    def nodeCaseMap = [:]
    def nodeIndex = 0

    // Init nodeCaseMap
    for(node in buildNodes){
        nodeCaseMap[node] = []
    }

    // each node add case
    for(caseName in cases){
        if(nodeIndex == buildNodes.size()){
            nodeIndex = 0
        }
        nodeCaseMap[buildNodes[nodeIndex]].add(caseName)
        nodeIndex = nodeIndex + 1
    }
    println "nodeCaseMap: ${nodeCaseMap}"
    // nodeCaseMap = ["qa_builder":["test_build.py::test_build[hpm6750evk2-sdk_gpio-ram_debug]", "test_build.py::test_build[hpm6750evk2-sdk_gpio-flash_xip]"]]
    def buildStages = [:]
    nodeCaseMap.each { buildNode, runCases ->
        buildStages[buildNode + "_build"] = {
            node(buildNode){
                stage(buildNode + "_build"){
                    deleteDir()
                    if(os == "linux"){
                        String casesStr = ""
                        if(runCases.size() != 0){
                            for(String caseName in runCases){
                                casesStr = casesStr + " " + caseName[0..-1]
                            }
                            def outputPath = "$WORKSPACE/$BUILD_ID/output"
                            def buildPath = "$WORKSPACE/rtt_build"
                            def nuttxPath = "$WORKSPACE/nuttx"
                            sh("$Globals.buildCodeClone && unzip -q /home/builder/${Globals.appsPackage}.zip && mv ${Globals.appsPackage} apps && git clone $Globals.nuttxCodeClone -b $branchName")
                            sh("export PATH=$PATH:/home/builder/nuttx_toolchain/riscv32-unknown-elf-newlib-multilib/bin && cd $buildPath/test_build/test_nuttx_build && /home/builder/.local/bin/pytest --suppress-tests-failed-exit-code --project nuttx $casesStr --project_src_dir $nuttxPath --build_dir $outputPath --junit-xml=$buildPath/report_${buildNode}.xml --jenkins_project $projectName --jenkins_build_id $BUILD_ID --jenkins_branch $BRANCH_NAME -p no:warnings")
                            
                            stash includes: "rtt_build/report_${buildNode}.xml", name:"report_${buildNode}"
                            Globals.linuxReportStash.put(buildNode, "report_${buildNode}")
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

    if(getProjectName().indexOf("nuttx") != -1){
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
        // replyTo: "${mailList}",
        recipientProviders: [[$class: 'CulpritsRecipientProvider']]
}// EmailNotification