# doble - The javadoc tool
# By: Evan Pratten <ewpratten@retrylife.ca>

use 5.010;
use strict;
use warnings;

# Handle CLI flags
my ($mode, $setting) = @ARGV;

# Ensure we actually have flags
if (not defined $mode){
    print "No program arguments specified. \nUse one of -l (local) or -p (publish) to specify mode.\n";
    exit 1;
} else {
    print "Starting doble\n";
}

# No matter the mode, we must build the documentation first
print "Building javadoc\n";
# system("./gradlew javadoc --console=plain");

print "Injecting javascript search bugfix\n";
system("sed -i 's/useModuleDirectories/false/g' build/reports/docs/search.js");

# Handle local hosting
if ($mode eq "-l"){
    print("Starting local javadoc server\n");
    print("This will move to the documentation directory\n");
    
    # Move to docs directory
    chdir "./build/reports/docs";

    # Start http server
    system("python -m SimpleHTTPServer 5806");

} elsif ($mode eq "-p"){
    print "Publishing documentation\n";

    # Read the current git origin URL
    my $origin = `git config --get remote.origin.url`;
    chomp $origin;

    # Read the repo name
    my $name = `basename \`git rev-parse --show-toplevel\``;
    chomp $name;

    print "Detected repo settings (Name: '$name', URL: '$origin')\n";

    # Clone a copy of the current repo, one level up
    chdir "../";
    system("git clone '$origin' '$name-DOBLE_TMP'");
    chdir "$name-DOBLE_TMP";

    # Checkout the latest gh-pages branch (Redundant)
    system("git checkout -b gh-pages");
    system("git checkout gh-pages");

    # Determine date string for branch name
    my $date = `date +"%I_%M-%h%d"`;
    chomp $date;

    # Checkout a new branch for this docs release
    system("git checkout -b javadoc-doble-$date");

    # Remove all branch contents
    system("rm -rf ./*");
    

} else {
    print "Invalid program arguments. Stopping\n";
    exit 1;

}